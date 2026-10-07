"""
Lap Analyzer Node:
Performance evaluation, real-time telemetry, lap timing, and RViz HUD visualization.
Decoupled observer monitoring /path and /state to compute cross-track error, heading error,
lap times, and dynamic metrics.

Lap detection uses cumulative (unwrapped) progress along the path instead of a
"s wrapped from >75% to <25%" heuristic. A lap is complete only when the car has
advanced one full track length along the path, so it cannot fire early.
"""

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class LapAnalyzer(Node):
    SEARCH_WINDOW = 80       # waypoints searched on each side of the last index
    RELOCALIZE_DIST = 3.0    # m: if the windowed match is farther, search globally
    def __init__(self):
        super().__init__('lap_analyzer')
        self.get_logger().info('Initializing Lap Analyzer Node...')

        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.state_sub = self.create_subscription(Odometry, '/state', self.state_callback, 10)

        # Publishers
        self.metrics_pub = self.create_publisher(String, '/lap/metrics', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/lap/visualization', 10)

        # Standardized Plottable Telemetry Publishers (for rqt_plot & PlotJuggler)
        self.cte_pub = self.create_publisher(Float32, '/telemetry/cte', 10)
        self.speed_pub = self.create_publisher(Float32, '/telemetry/speed', 10)
        self.heading_err_pub = self.create_publisher(
            Float32, '/telemetry/heading_err_deg', 10
        )
        self.lap_time_pub = self.create_publisher(Float32, '/telemetry/lap_time', 10)

        # Path storage (closed loop: segment i goes from point i to point (i+1) % n)
        self.path_points = []  # [(x, y, psi)]
        self.xs = None
        self.ys = None
        self.path_cum_dist = []  # n + 1 entries, last one is the full perimeter
        self.track_length = 0.0
        self.path_received = False
        self.raw_path_len = 0
        self.last_idx = None

        # Timing
        self.last_state_time = None
        self.lap_start_time = None

        # Progress tracking (unwrapped arc length along the path)
        self.prev_s = None
        self.progress = 0.0
        self.prev_progress = 0.0
        self.next_lap_progress = None

        # Odometers
        self.total_distance = 0.0
        self.lap_distance = 0.0
        self.last_xy = None

        # Laps
        self.lap_count = 0
        self.current_lap_time = 0.0
        self.last_lap_time = None
        self.best_lap_time = None
        self.lap_times = []
        self.lap_stats = []

        # Per-lap samples
        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

        # Global statistics
        self.global_ctes = []
        self.global_max_speed = 0.0

        # Current live metrics
        self.current_cte = 0.0
        self.current_heading_err = 0.0
        self.current_speed = 0.0
        self.proj_xy = (0.0, 0.0)

        # Publish telemetry and HUD at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_telemetry)

    # ------------------------------------------------------------------
    # Path handling
    # ------------------------------------------------------------------
    def path_callback(self, msg: Path):
        """Processes received path and precomputes cumulative distance (closed loop)."""
        if self.path_received and len(msg.poses) == self.raw_path_len:
            return  # Path already loaded and unchanged

        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            qz = p.pose.orientation.z
            qw = p.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            pts.append((x, y, yaw))

        if len(pts) < 3:
            return

        # If the loader already appended the first waypoint at the end, drop the
        # duplicate so the closing segment is counted exactly once.
        if math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-3:
            pts = pts[:-1]

        n = len(pts)
        cum = [0.0]
        for i in range(1, n + 1):
            a = pts[i - 1]
            b = pts[i % n]  # i == n closes the loop back to point 0
            cum.append(cum[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

        self.path_points = pts
        self.xs = np.array([p[0] for p in pts])
        self.ys = np.array([p[1] for p in pts])
        self.path_cum_dist = cum
        self.track_length = cum[-1]
        self.raw_path_len = len(msg.poses)
        self.path_received = True

        # A new path invalidates all progress tracking
        self.last_idx = None
        self.prev_s = None

        self.get_logger().info(
            f"Lap Analyzer: loaded path with {n} unique waypoints "
            f"({len(msg.poses)} received), closed perimeter: {self.track_length:.2f} m"
        )

    def project_to_path(self, x, y, yaw):
        """Projects (x, y) onto the closed path. Returns projection, arc length s,
        signed CTE and heading error. Uses a local search window once localized so the
        arc length can never jump to a different part of the track."""
        pts = self.path_points
        n = len(pts)

        nearest_idx = None
        if self.last_idx is not None:
            offsets = np.arange(-self.SEARCH_WINDOW, self.SEARCH_WINDOW + 1)
            idxs = (self.last_idx + offsets) % n
            d2 = (self.xs[idxs] - x) ** 2 + (self.ys[idxs] - y) ** 2
            k = int(np.argmin(d2))
            if math.sqrt(float(d2[k])) <= self.RELOCALIZE_DIST:
                nearest_idx = int(idxs[k])

        if nearest_idx is None:  # first fix, or the car got lost: global search
            d2 = (self.xs - x) ** 2 + (self.ys - y) ** 2
            nearest_idx = int(np.argmin(d2))
            if self.last_idx is not None:
                self.get_logger().warn(
                    'Lap Analyzer: relocalized globally at x=%.1f y=%.1f (idx %d -> %d)'
                    % (x, y, self.last_idx, nearest_idx))
        self.last_idx = nearest_idx

        best_dist = float('inf')
        best_proj = (pts[nearest_idx][0], pts[nearest_idx][1])
        best_s = self.path_cum_dist[nearest_idx]
        best_seg_yaw = pts[nearest_idx][2]
        best_signed_cte = 0.0

        for prev_i, next_i in (((nearest_idx - 1) % n, nearest_idx),
                               (nearest_idx, (nearest_idx + 1) % n)):
            x1, y1, _ = pts[prev_i]
            x2, y2, _ = pts[next_i]
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-9:
                continue

            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg_len_sq))
            px = x1 + t * dx
            py = y1 + t * dy
            dist = math.hypot(x - px, y - py)

            if dist < best_dist:
                best_dist = dist
                best_proj = (px, py)
                best_s = self.path_cum_dist[prev_i] + t * math.sqrt(seg_len_sq)
                best_seg_yaw = math.atan2(dy, dx)
                # Signed cross-track error: positive if the car is left of the path
                cross = dx * (y - y1) - dy * (x - x1)
                best_signed_cte = math.copysign(dist, cross)

        heading_err = math.atan2(math.sin(yaw - best_seg_yaw), math.cos(yaw - best_seg_yaw))
        return best_proj[0], best_proj[1], best_s, best_signed_cte, heading_err

    # ------------------------------------------------------------------
    # State handling
    # ------------------------------------------------------------------
    def state_callback(self, msg: Odometry):
        """Processes vehicle odometry and updates progress, lap timing, and errors."""
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x

        self.current_speed = v
        self.global_max_speed = max(self.global_max_speed, v)

        # Odometers (position differences)
        if self.last_xy is not None:
            step_d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.total_distance += step_d
            self.lap_distance += step_d
        self.last_xy = (x, y)

        if not self.path_received or len(self.path_points) < 3:
            self.last_state_time = now_sec
            return

        proj_x, proj_y, s, cte, heading_err = self.project_to_path(x, y, yaw)
        self.proj_xy = (proj_x, proj_y)
        self.current_cte = cte
        self.current_heading_err = heading_err

        L = self.track_length

        if self.prev_s is None:
            # First valid fix: start the first lap here.
            # If the car sits just behind the start line, s is close to L, so use a
            # negative starting progress.
            self.progress = s if s <= 0.5 * L else s - L
            self.prev_progress = self.progress
            self.next_lap_progress = L
            self.lap_start_time = now_sec
            self.lap_distance = 0.0
            self.lap_ctes = []
            self.lap_heading_errors = []
            self.lap_speeds = []
        else:
            ds = s - self.prev_s
            ds = (ds + 0.5 * L) % L - 0.5 * L  # wrap into [-L/2, L/2)
            self.prev_progress = self.progress
            self.progress += ds
        self.prev_s = s

        # Accumulate metrics
        abs_cte = abs(cte)
        self.lap_ctes.append(abs_cte)
        self.lap_heading_errors.append(abs(heading_err))
        self.lap_speeds.append(v)
        self.global_ctes.append(abs_cte)

        self.current_lap_time = now_sec - self.lap_start_time

        # Lap completes when cumulative forward progress reaches the next full lap
        if self.progress >= self.next_lap_progress:
            den = self.progress - self.prev_progress
            if den > 1e-9:
                frac = (self.next_lap_progress - self.prev_progress) / den
            else:
                frac = 1.0
            frac = max(0.0, min(1.0, frac))
            t_prev = self.last_state_time if self.last_state_time is not None else now_sec
            t_cross = t_prev + frac * (now_sec - t_prev)

            self.record_lap_completion(t_cross - self.lap_start_time)
            self.lap_start_time = t_cross
            self.next_lap_progress += L

        self.last_state_time = now_sec

    def record_lap_completion(self, lap_duration):
        """Records finished lap and prints summary."""
        self.lap_count += 1
        self.last_lap_time = lap_duration
        self.lap_times.append(lap_duration)

        if self.best_lap_time is None or lap_duration < self.best_lap_time:
            self.best_lap_time = lap_duration

        ctes = np.array(self.lap_ctes) if self.lap_ctes else np.zeros(1)
        speeds = np.array(self.lap_speeds) if self.lap_speeds else np.zeros(1)
        herrs = np.array(self.lap_heading_errors) if self.lap_heading_errors else np.zeros(1)

        mean_cte = float(np.mean(ctes))
        max_cte = float(np.max(ctes))
        rms_cte = float(np.sqrt(np.mean(ctes ** 2)))
        mean_speed = float(np.mean(speeds))
        max_speed = float(np.max(speeds))
        mean_herr_deg = float(np.degrees(np.mean(herrs)))

        L = self.track_length
        lap_odo = self.lap_distance
        total_track_distance = L * self.lap_count

        self.lap_stats.append({
            'lap': self.lap_count, 'lap_time': lap_duration,
            'mean_cte': mean_cte, 'max_cte': max_cte, 'rms_cte': rms_cte,
            'mean_speed': mean_speed, 'max_speed': max_speed,
            'mean_heading_err_deg': mean_herr_deg, 'lap_odometer': lap_odo,
        })

        lines = [
            '=================== LAP %d COMPLETE ===================' % self.lap_count,
            '  Lap time       : %.2f s (best: %.2f s)' % (lap_duration, self.best_lap_time),
            '  CTE            : mean %.3f | RMS %.3f | max %.3f m' % (
                mean_cte, rms_cte, max_cte),
            '  Heading error  : mean %.2f deg' % mean_herr_deg,
            '  Speed          : mean %.2f m/s | max %.2f m/s' % (mean_speed, max_speed),
        ]
        self.get_logger().info('\n' + '\n'.join(lines))

        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []
        self.lap_distance = 0.0

    def print_summary(self):
        """Prints a benchmark table at shutdown (copy it into the README)."""
        if not self.lap_stats:
            return
        header = ('Lap | Time (s) | Mean CTE | RMS CTE | Max CTE | '
                  'Mean v | Max v | Odometer (m)')
        rows = [header, '-' * len(header)]
        for s in self.lap_stats:
            rows.append('%3d | %8.2f | %8.3f | %7.3f | %7.3f | %6.2f | %5.2f | %8.1f' % (
                s['lap'], s['lap_time'], s['mean_cte'], s['rms_cte'], s['max_cte'],
                s['mean_speed'], s['max_speed'], s['lap_odometer']))
        rows.append('Best lap: %.2f s | Top speed: %.2f m/s | Laps: %d' % (
            self.best_lap_time, self.global_max_speed, self.lap_count))
        print('\n' + '\n'.join(rows) + '\n')


    # ------------------------------------------------------------------
    # Telemetry and visualization
    # ------------------------------------------------------------------
    def publish_telemetry(self):
        """Periodically publishes numerical telemetry and RViz visual markers at 10 Hz."""
        self.cte_pub.publish(Float32(data=float(self.current_cte)))
        self.speed_pub.publish(Float32(data=float(self.current_speed)))
        self.heading_err_pub.publish(
            Float32(data=float(math.degrees(self.current_heading_err))))
        self.lap_time_pub.publish(Float32(data=float(self.current_lap_time)))

        if self.lap_ctes:
            rms = math.sqrt(sum(c * c for c in self.lap_ctes) / len(self.lap_ctes))
        else:
            rms = 0.0
        telemetry = {
            'lap': self.lap_count,
            'current_lap_time': round(self.current_lap_time, 3),
            'last_lap_time': self.last_lap_time,
            'best_lap_time': self.best_lap_time,
            'speed': round(self.current_speed, 3),
            'current_cte': round(self.current_cte, 4),
            'rms_cte': round(rms, 4),
            'heading_err_deg': round(math.degrees(self.current_heading_err), 3),
        }
        self.metrics_pub.publish(String(data=json.dumps(telemetry)))
        self.publish_rviz_markers(telemetry)

    def publish_rviz_markers(self, telemetry=None):
        """Renders start gate, error whisker, and on-screen HUD text in RViz."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Marker 1: start/finish gate at the track origin
        if self.path_points:
            p0 = self.path_points[0]
            gate = Marker()
            gate.header.frame_id = 'map'
            gate.header.stamp = now
            gate.ns = 'start_gate'
            gate.id = 0
            gate.type = Marker.CYLINDER
            gate.action = Marker.ADD
            gate.pose.position.x = p0[0]
            gate.pose.position.y = p0[1]
            gate.pose.position.z = 0.5
            gate.pose.orientation.w = 1.0
            gate.scale.x = 0.1
            gate.scale.y = 1.2
            gate.scale.z = 1.0
            gate.color.r = 0.1
            gate.color.g = 0.9
            gate.color.b = 0.2
            gate.color.a = 0.7
            ma.markers.append(gate)

        # Marker 2: CTE whisker (green when tracking well, red as error nears 1 m)
        if self.last_xy is not None and self.path_received:
            err = min(abs(self.current_cte), 1.0)
            whisker = Marker()
            whisker.header.frame_id = 'map'
            whisker.header.stamp = now
            whisker.ns = 'cte_whisker'
            whisker.id = 1
            whisker.type = Marker.LINE_STRIP
            whisker.action = Marker.ADD
            whisker.scale.x = 0.1
            whisker.color.r = err
            whisker.color.g = 1.0 - err
            whisker.color.b = 0.0
            whisker.color.a = 1.0
            whisker.points = [
                Point(x=float(self.last_xy[0]), y=float(self.last_xy[1]), z=0.2),
                Point(x=float(self.proj_xy[0]), y=float(self.proj_xy[1]), z=0.2),
            ]
            ma.markers.append(whisker)

        # Marker 3: floating scoreboard above the car
        if telemetry is not None and self.last_xy is not None:
            best = telemetry['best_lap_time']
            best_txt = '%.2f s' % best if best is not None else '--'
            hud = Marker()
            hud.header.frame_id = 'map'
            hud.header.stamp = now
            hud.ns = 'hud'
            hud.id = 2
            hud.type = Marker.TEXT_VIEW_FACING
            hud.action = Marker.ADD
            hud.pose.position.x = float(self.last_xy[0])
            hud.pose.position.y = float(self.last_xy[1])
            hud.pose.position.z = 3.0
            hud.pose.orientation.w = 1.0
            hud.scale.z = 0.8
            hud.color.r = hud.color.g = hud.color.b = hud.color.a = 1.0
            hud.text = 'Lap %d | %.1f s\nSpeed %.1f m/s\nCTE %+.2f m\nBest %s' % (
                telemetry['lap'], telemetry['current_lap_time'], telemetry['speed'],
                telemetry['current_cte'], best_txt)
            ma.markers.append(hud)

        self.viz_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    analyzer = LapAnalyzer()
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.print_summary()
        analyzer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
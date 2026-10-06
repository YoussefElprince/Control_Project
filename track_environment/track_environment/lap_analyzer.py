"""
Lap Analyzer Node:
Performance evaluation, real-time telemetry, lap timing, and RViz HUD visualization.
Decoupled observer monitoring /path and /state to compute cross-track error, heading error,
lap times, and dynamic metrics.
"""

import json  # noqa: F401
import math
import numpy as np  # noqa: F401
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Float32  # noqa: F401
from geometry_msgs.msg import Point  # noqa: F401
from visualization_msgs.msg import Marker, MarkerArray


class LapAnalyzer(Node):
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

        # Path storage
        self.path_points = []  # [(x, y, psi)]
        self.path_cum_dist = []
        self.track_length = 0.0
        self.path_received = False

        # State & Timing
        self.start_sim_time = None
        self.last_state_time = None
        self.lap_start_time = None

        # Lap Tracking
        self.lap_count = 0
        self.last_s = 0.0
        self.total_distance = 0.0
        self.last_xy = None

        # Lap Times
        self.current_lap_time = 0.0
        self.last_lap_time = None
        self.best_lap_time = None
        self.lap_times = []

        # Error & Speed Statistics (Per Lap)
        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

        # Global Statistics
        self.global_ctes = []
        self.global_max_speed = 0.0

        # Current live metrics
        self.current_cte = 0.0
        self.current_heading_err = 0.0
        self.current_speed = 0.0
        self.proj_xy = (0.0, 0.0)

        # Publish periodic summary and HUD at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_telemetry)

    def path_callback(self, msg: Path):
        """Processes received path and precomputes cumulative distance."""
        if self.path_received and len(msg.poses) == len(self.path_points):
            return  # Path already loaded and unchanged

        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            # Extract yaw from quaternion
            qz = p.pose.orientation.z
            qw = p.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            pts.append((x, y, yaw))

        if len(pts) < 2:
            return

        self.path_points = pts
        # Compute cumulative distance
        cum = [0.0]
        for i in range(1, len(pts)):
            d = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            cum.append(cum[-1] + d)

        self.path_cum_dist = cum
        self.track_length = cum[-1]
        self.path_received = True
        self.get_logger().info(
            f"Lap Analyzer: Loaded path with {len(pts)} waypoints, "
            f"perimeter: {self.track_length:.2f} m"
        )

    def state_callback(self, msg: Odometry):
        """Processes vehicle odometry and updates progress, lap timing, and errors."""
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_sim_time is None:
            self.start_sim_time = now_sec
            self.lap_start_time = now_sec

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x

        self.current_speed = v
        self.global_max_speed = max(self.global_max_speed, v)

        # Distance traveled
        if self.last_xy is not None:
            step_d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.total_distance += step_d
        self.last_xy = (x, y)

        if not self.path_received or len(self.path_points) < 2:
            return

        # Find nearest point & projection
        proj_x, proj_y, s, cte, heading_err = self.project_to_path(x, y, yaw)
        self.proj_xy = (proj_x, proj_y)
        self.current_cte = cte
        self.current_heading_err = heading_err

        # Accumulate metrics
        abs_cte = abs(cte)
        self.lap_ctes.append(abs_cte)
        self.lap_heading_errors.append(abs(heading_err))
        self.lap_speeds.append(v)
        self.global_ctes.append(abs_cte)

        self.current_lap_time = now_sec - self.lap_start_time

        # Lap Crossing Detection (s wrapped around track_length while moving forward).
        # e.g., last_s near end (> 70% length) and current s near start (< 30% length).
        if self.track_length > 5.0 and v > 0.1:
            if self.last_s > 0.75 * self.track_length and s < 0.25 * self.track_length:
                # Sub-tick lap time interpolation
                ds_total = (self.track_length - self.last_s) + s
                dt_step = max(now_sec - (self.last_state_time or now_sec), 1e-4)
                frac = (self.track_length - self.last_s) / max(ds_total, 1e-4)
                t_crossing = (self.last_state_time or now_sec) + frac * dt_step

                lap_duration = t_crossing - self.lap_start_time
                self.record_lap_completion(lap_duration, now_sec)
                self.lap_start_time = t_crossing

        self.last_s = s
        self.last_state_time = now_sec

    def project_to_path(self, x, y, yaw):
        """Finds closest segment and projects (x, y) to compute exact orthogonal CTE."""
        pts = self.path_points
        n = len(pts)

        # 1. Find nearest waypoint
        min_dist_sq = float('inf')
        nearest_idx = 0
        for i in range(n):
            dx = pts[i][0] - x
            dy = pts[i][1] - y
            d_sq = dx * dx + dy * dy
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                nearest_idx = i

        # 2. Check candidate segments: (prev, nearest) and (nearest, next)
        best_dist = float('inf')
        best_proj = (pts[nearest_idx][0], pts[nearest_idx][1])
        best_s = self.path_cum_dist[nearest_idx]
        best_seg_yaw = pts[nearest_idx][2]
        best_signed_cte = 0.0

        candidate_segments = [
            ((nearest_idx - 1) % n, nearest_idx),
            (nearest_idx, (nearest_idx + 1) % n)
        ]
        for prev_i, next_i in candidate_segments:
            x1, y1, yaw1 = pts[prev_i]
            x2, y2, _ = pts[next_i]
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-6:
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

                # Signed cross track error: positive if car is to the left of path
                cross = dx * (y - y1) - dy * (x - x1)
                best_signed_cte = math.copysign(dist, cross)

        # Heading error in [-pi, pi]
        heading_err = math.atan2(math.sin(yaw - best_seg_yaw), math.cos(yaw - best_seg_yaw))

        return best_proj[0], best_proj[1], best_s, best_signed_cte, heading_err

    def record_lap_completion(self, lap_duration, now_sec):
        """Records finished lap and prints summary."""
        self.lap_count += 1
        self.last_lap_time = lap_duration
        self.lap_times.append(lap_duration)

        if self.best_lap_time is None or lap_duration < self.best_lap_time:
            self.best_lap_time = lap_duration
        # ======================================================================
        ctes = np.array(self.lap_ctes) if self.lap_ctes else np.zeros(1)
        speeds = np.array(self.lap_speeds) if self.lap_speeds else np.zeros(1)

        mean_cte = float(np.mean(ctes))
        max_cte = float(np.max(ctes))
        rms_cte = float(np.sqrt(np.mean(ctes ** 2)))
        mean_speed = float(np.mean(speeds))
        max_speed = float(np.max(speeds))

        self.lap_stats.append({
            'lap': self.lap_count, 'lap_time': lap_duration,
            'mean_cte': mean_cte, 'max_cte': max_cte, 'rms_cte': rms_cte,
            'mean_speed': mean_speed, 'max_speed': max_speed,
        })

        lines = [
            '=================== LAP %d COMPLETE ===================' % self.lap_count,
            '  Lap time : %.2f s (best: %.2f s)' % (lap_duration, self.best_lap_time),
            '  CTE      : mean %.3f | RMS %.3f | max %.3f m' % (mean_cte, rms_cte, max_cte),
            '  Speed    : mean %.2f m/s | max %.2f m/s' % (mean_speed, max_speed),
            '  Distance : %.1f m' % self.total_distance,
        ]
        self.get_logger().info('\n' + '\n'.join(lines))

        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

    def publish_telemetry(self):
        """Periodically publishes numerical telemetry and RViz visual markers at 10 Hz."""
        # ======================================================================
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

        # ----------------------------------------------------------------------
        # Marker 1: Start/Finish Gate Line (Provided for Track Origin Reference)
        # ----------------------------------------------------------------------
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
        # ======================================================================
        if self.last_xy is not None and self.path_received:
            # CTE whisker: green when tracking well, red as the error approaches 1 m
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

        if telemetry is not None and self.last_xy is not None:
            # Floating scoreboard above the car
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
        analyzer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

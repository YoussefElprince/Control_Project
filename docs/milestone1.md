# Milestone 1: Discovery

## 1. Setup
- OS: Ubuntu 22.04 (WSL2), ROS 2 Humble
- Workspace: `~/control_ros2_ws`, packages: `bicycle_sim`, `bicycle_control`, `track_environment`
- Launch command: `ros2 launch bicycle_sim bicycle_sim.launch.py`

## 2. Nodes
| Node | Role | Subscribes | Publishes |
|---|---|---|---|
| /kinematic_bicycle | Vehicle simulator | /throttle, /steer | /state, /joint_states, /tf |
| /path_gen | Loads track CSV | none | /path, /track_bounds |
| /lap_analyzer | Lap timing and telemetry | /path, /state | /telemetry/*, /lap/metrics, /lap/visualization |
| /robot_state_publisher | Robot model and transforms | /joint_states | /robot_description, /tf, /tf_static |
| /rviz2 | Visualization | /tf, /path, markers | none |

## 3. Topics, message types and units
| Topic | Message type | Unit / meaning |
|---|---|---|
| /throttle | std_msgs/msg/Float32 | normalised throttle and brake, range [-1.0, 1.0] |
| /steer | std_msgs/msg/Float32 | front steering angle in rad, positive = left |
| /state | nav_msgs/msg/Odometry | position x, y in m; yaw as quaternion; speed in m/s (TODO: confirm fields with `ros2 topic echo /state --once`) |
| /path | nav_msgs/msg/Path | track centerline, positions in m |
| /track_bounds | visualization_msgs/msg/MarkerArray | boundary cones |
| /joint_states | sensor_msgs/msg/JointState | wheel and steering joint values for the model |
| /telemetry/speed | std_msgs/msg/Float32 | m/s |
| /telemetry/cte | std_msgs/msg/Float32 | cross-track error, m |
| /telemetry/heading_err_deg | std_msgs/msg/Float32 | heading error, degrees |
| /telemetry/lap_time | std_msgs/msg/Float32 | s |
| /lap/metrics | std_msgs/msg/String | text summary of lap statistics |
| /lap/visualization | visualization_msgs/msg/MarkerArray | RViz dashboard markers |

RViz also adds /goal_pose, /clicked_point and /initialpose. The project does not use them.

## 4. Data flow
/throttle and /steer go into /kinematic_bicycle, which integrates the vehicle model and publishes /state. /lap_analyzer compares /state against /path to compute cross-track error, heading error, speed and lap time, and republishes them as telemetry topics and RViz markers. /joint_states drives the wheel and steering motion in RViz through /robot_state_publisher.

Nothing publishes /throttle or /steer yet. The teleop bridge (M3), the cruise PID (M4) and the lateral controllers (M5) will fill that gap. Because /lap_analyzer only reads /state and /path, the controllers can be swapped without changing the analyzer, which keeps the benchmark fair.

## 5. Vehicle and track parameters (from the project description)
- Wheelbase 1.25 m, track width 1.18 m, wheel radius 0.5 m, wheel width 0.3 m
- Track: 1000 centerline waypoints, first at (0, 0), closed loop, perimeter about 528.2 m

## 6. Rates
- /state publish rate: TODO Hz (from `ros2 topic hz /state`), so dt = TODO s

## 7. Live plotting
- rqt_plot: initially failed with a NumPy 2.x vs system matplotlib incompatibility, fixed by pinning `numpy==1.26.4`
- PlotJuggler: set up with the ROS2 Topic Subscriber, plotting /telemetry/speed and /telemetry/cte

## 8. Observations
- TODO: what happened to /telemetry/speed and /state when publishing /throttle at 0.5 and 10 Hz (speed rises and saturates, or stays at 0 because the dynamics are still a stub)
EOF

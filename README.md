# Behavior-Level Autonomous Navigation with Perception–Control Arbitration (ROS)

## Overview
# Final Result
![sample_results](Videos/Turtlebot3_robot_trajectory_follower.gif)

This project implements a **behavior-level autonomy stack** for a mobile robot using ROS.
It combines waypoint-based motion control with a **reactive LiDAR safety layer** and a
deterministic **velocity arbiter / finite-state behavior layer**.

The system cleanly separates **motion generation**, **safety enforcement**, and
**command arbitration**, allowing the robot to stop, slow, or proceed safely in the
presence of obstacles while remaining debuggable, testable, and extensible.

## Key Features

```
Controller Node ──► /desired_cmd_vel ──┐
├─► Velocity Arbiter ──► /cmd_vel
LiDAR Safety Node ─► /safety_cmd_vel ──┘
▲
│
/robot_state
Forward-arc LiDAR obstacle detection
Distance-based stopping and speed scaling
Hysteresis to prevent oscillatory behavior
Time-To-Collision (TTC) based braking
Velocity arbitration between controller and safety layer
RViz visualization of safety arc and waypoints
Runtime-tunable ROS parameters
```

**Demo:**
```bash
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch ros-closed-loop-controller-node controller.launch
```
## Node Architecture

### 1. Controller Node (controller_node.py)

```
Generates waypoint-based motion (square / figure-8)
Publishes desired velocity on /desired_cmd_vel
Pure motion logic (no obstacle awareness)
Publishes waypoint markers for RViz
```
### 2. LiDAR Safety Node (lidar_obstacle_mgmt.py)

```
Subscribes to /scan
Extracts front-arc LiDAR data
Computes minimum obstacle distance
Estimates Time-To-Collision (TTC) using current speed
Applies distance + TTC based speed limiting
Publishes safety velocity on /safety_cmd_vel
Visualizes front safety arc in RViz
```
### 3. Velocity Arbiter (lidar_movement_arbiter.py)

```
Subscribes to /desired_cmd_vel and /safety_cmd_vel
Publishes final velocity on /cmd_vel
Always enforces safety by limiting linear speed
Publishes high-level robot state on /robot_state
States: IDLE, MOVE, SLOW, STOP
```
#### • • • • • • • • • • • • • • • • • •


## Safety Logic Summary

The robot’s forward velocity is determined using **distance** and
**time-to-collision (TTC)** metrics.

### Hard Stop
- Obstacle distance ≤ effective_stop  
- OR TTC ≤ ttc_stop  

### Full Speed
- Obstacle distance ≥ safe_distance  
- AND TTC ≥ ttc_slow  

### Scaled Speed
- Otherwise, speed is scaled using the minimum of:
  - Distance-based scaling
  - TTC-based scaling

This ensures smooth deceleration and prevents late braking at higher speeds.

## Parameters (ROS)

| Parameter        | Description                              | Example |
|------------------|------------------------------------------|---------|
| stop_distance    | Base stopping distance                   | 0.6     |
| safe_distance    | Distance to resume full speed            | 1.2     |
| moving_speed     | Maximum allowed forward speed            | 0.6     |
| front_min_angle  | Left bound of front arc (degrees)        | -30     |
| front_max_angle  | Right bound of front arc (degrees)       | 30      |
| ttc_stop         | Emergency stop TTC threshold (seconds)   | 0.8     |
| ttc_slow         | Resume TTC threshold (seconds)           | 1.5     |

All parameters are configurable via launch files.

## Visualization

```
Front safety arc displayed using visualization_msgs/Marker
Waypoints rendered in RViz
Color-coded arc indicates stop vs safe state

## rosbag Support
Records /cmd_vel, /safety_cmd_vel, /robot_state, /odom, /scan
Allows offline debugging and deterministic replay
Supports behavior validation without rerunning simulation
```
#### • • • • • • • • •


## Design Decisions

```
Safety always overrides motion commands
Reactive logic favored over complex prediction
Velocity arbitration instead of command cancellation
Explicit state publication for observability
Deterministic timers for repeatable behavior
```
This keeps the system robust, simple, and easy to debug.

## Limitations (Intentional)

```
No prediction along curved trajectories
No global or local planner integration
No dynamic obstacle tracking
```
These are addressed in the next project.

## Next Planned Extension

**Predictive Local Planner with Trajectory Collision Checking** - 
Forward simulation of motion - Curvature-aware safety checks - Planner-aware braking and re-planning

## Skills Demonstrated

```
ROS node architecture and message flow design
LiDAR data processing and filtering
Real-time safety and TTC-based control logic
Velocity arbitration and behavior-level autonomy
Simulation debugging and rosbag-based validation
```
## Status

✅ Complete and stable 🚀 Extended in next project

#### • • • • • • • • • • •




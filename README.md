# LiDAR-Based Safety Controller with Arbiter (ROS)

## Overview

This project implements a **reactive LiDAR-based safety layer** for a mobile robot using ROS. The system
monitors obstacles in the robot’s forward arc and dynamically **stops, slows, or allows motion** based on
distance and time-to-collision (TTC) metrics.

The safety logic is **decoupled** from motion planning via a velocity arbiter, making the architecture modular,
debuggable, and extensible.

## Key Features

```
Forward-arc LiDAR obstacle detection
Distance-based stopping and speed scaling
Hysteresis to prevent oscillatory behavior
Time-To-Collision (TTC) based braking
Velocity arbitration between controller and safety layer
RViz visualization of safety arc and waypoints
Runtime-tunable ROS parameters
```
## Node Architecture

### 1. Controller Node (controller_node.py)

```
Generates waypoint-based motion (square / figure-8)
Publishes desired velocity on /desired_cmd_vel
Pure motion logic (no obstacle awareness)
```
### 2. LiDAR Safety Node (lidar_obstacle_mgmt.py)

```
Subscribes to /scan
Computes minimum distance in front arc
Estimates Time-To-Collision (TTC)
Publishes safety-limited velocity on /safety_cmd_vel
Visualizes monitored front arc in RViz
```
### 3. Velocity Arbiter (lidar_movement_arbiter.py)

```
Subscribes to /desired_cmd_vel and /safety_cmd_vel
Publishes final /cmd_vel
Enforces safety by limiting linear velocity
```
#### • • • • • • • • • • • • • • • • • •


## Safety Logic Summary

The robot’s forward speed is determined by: - **Hard Stop** if: - Obstacle is closer than effective_stop, OR

- TTC is below ttc_stop

```
Full Speed if:
Obstacle is beyond safe_distance, AND
```
```
TTC is above ttc_slow
```
```
Scaled Speed otherwise, using the minimum of:
```
```
Distance-based scaling
TTC-based scaling
```
This ensures smooth deceleration and prevents late braking at higher speeds.

## Parameters (ROS)

```
Parameter Description Example
```
```
stop_distance Base stopping distance 0.
```
```
safe_distance Distance to resume full speed 1.
```
```
moving_speed Max allowed forward speed 0.
```
```
front_min_angle Left bound of front arc (deg) -
```
```
front_max_angle Right bound of front arc (deg) 30
```
```
ttc_stop Emergency stop TTC (s) 1.
```
```
ttc_slow Resume TTC (s) 2.
```
All parameters are configurable via launch files.

## Visualization

```
Front safety arc displayed using visualization_msgs/Marker
Waypoints rendered in RViz
Color-coded arc indicates stop vs safe state
```
#### • • • • • • • • •


## Design Decisions

```
Safety logic is reactive , not predictive
No trajectory rollout or curvature prediction
TTC added as a lightweight improvement without planner coupling
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

**Predictive Local Planner with Trajectory Collision Checking** - Forward simulation of motion - Curvature-
aware safety checks - Planner-aware braking and re-planning

## Skills Demonstrated

```
ROS node architecture & message flow
LiDAR processing and filtering
Real-time safety logic
Velocity arbitration
Debugging simulation timing and control issues
```
## Status

✅ Complete and stable 🚀 Extended in next project

#### • • • • • • • • • • •




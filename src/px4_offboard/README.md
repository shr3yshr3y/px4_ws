# px4_offboard

PX4 Offboard Control ROS 2 Package

## Overview
This package provides a ROS 2 node for teleoperation and offboard control of PX4-based drones using keyboard input. It leverages MAVROS and ROS 2 to send position setpoints and mode/arming commands to the drone.

## Features
- Keyboard teleoperation for drone movement in X, Y, Z axes
- Commands for arming, disarming, switching to OFFBOARD and LAND modes
- Publishes position setpoints to `/mavros/setpoint_position/local`
- Subscribes to drone state via `/mavros/state`
- Service clients for arming/disarming and mode switching
- Example node: `control_node.py`

## Usage
Run the node with:

```
ros2 run px4_offboard control_node
```

Follow the on-screen instructions to control the drone using your keyboard.

## File Structure
- `px4_offboard/control_node.py`: Main teleop node
- `px4_offboard/__init__.py`: Package init (empty)
- `setup.py`, `setup.cfg`: Build configuration
- `package.xml`: ROS 2 package manifest
- `test/`: Lint and copyright tests

## Requirements
- ROS 2
- MAVROS
- PX4-compatible drone
- Python 3.12

## Maintainer
- shreyas (sgspouli@liverpool.ac.uk)

## License
See `package.xml` for license information.

# PX4 Workspace

This workspace contains the `px4_offboard` ROS 2 package for teleoperation and offboard control of PX4 drones.

## Structure
- `src/px4_offboard/`: Main package source
    - `px4_offboard/control_node.py`: Teleop node
    - `test/`: Lint and copyright tests
    - `setup.py`, `setup.cfg`, `package.xml`: Build and manifest files
- `build/`, `install/`, `log/`: Build and install artifacts

## Quick Start
1. Build the workspace:
   ```
   colcon build
   ```
2. Source the setup file:
   ```
   source install/setup.bash
   ```
3. Run the teleop node:
   ```
   ros2 run px4_offboard control_node
   ```

## Requirements
- ROS 2
- MAVROS
- PX4-compatible drone
- Python 3.12

## Maintainer
- shreyas (sgspouli@liverpool.ac.uk)

## License
See `src/px4_offboard/package.xml` for license information.

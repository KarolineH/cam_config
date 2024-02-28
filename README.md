## Custom ROS2 Camera Configuration Package

This package contains custom configurations and calibration data for a CANON EOS R5 C mounted to a Kinova Gen3 robot arm.
Note that this is specific to our lens, custom printed mounting plate, and environment.
It publishes a static transform between robot wrist (tool frame) and camera frame.

### Prerequisites
- Tested with ROS2 Iron
- Creat your own colcon workspace ([instructions](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html))
- Clone this package into your workspace/src directory
- Navigate to your workspace directory and run `colcon build`
- Finally, source your workspace `source ./install/setup.bash`

### Usage
- Make sure the frame parameters are set correctly in config/frame_transform.yaml
- Run the broadcaster with `ros2 run cam_config static_camera_tf2_broadcaster`
Current package install files: 
sudo apt install \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-ros-gz \
  ros-jazzy-tf2-ros \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-localization \
  ros-jazzy-slam-toolbox \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-gz-ros2-control


sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces


ros2 run xacro xacro src/robot/urdf/robot_model.xacro -o src/robot/urdf/robot_fin.urdf
colcon build
source install/setup.bash
ros2 launch ros_sim_robot spawn_robot.launch.py

ros2 topic list
 gz topic -l 

ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw


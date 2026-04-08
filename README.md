Current package install files: 
sudo apt install ros-jazzy-xacro 
sudo apt install ros-jazzy-robot-state-publisher 
sudo apt install ros-jazzy-joint-state-publisher 
sudo apt install ros-jazzy-ros2-control 
sudo apt install  ros-jazzy-ros2-controllers 
sudo apt install  ros-jazzy-ros-gz 
sudo apt install  ros-jazzy-tf2-ros 
sudo apt install  ros-jazzy-rviz2 
sudo apt install  ros-jazzy-robot-localization 
sudo apt install  ros-jazzy-slam-toolbox 
sudo apt install  ros-jazzy-joint-state-publisher-gui 
sudo apt install  ros-jazzy-gz-ros2-control


sudo apt install ros-jazzy-ros-gz
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces

Build and run instructions \
colcon build --packages-select ros_sim_robot \
source install/setup.bash \
ros2 run xacro xacro src/robot/urdf/robot_model.xacro -o src/robot/urdf/robot_fin.urdf \
Rebuild Again after urdf generation! /
colcon build --packages-select ros_sim_robot \
source install/setup.bash \
ros2 launch ros_sim_robot spawn_robot.launch.py \

To View Camera output \

ros2 run image_tools showimage --ros-args -r image:=/camera/image_raw \ 



Video Link: https://drive.google.com/file/d/1OKP1u_MqxAMWfpbAgNeyYjOmiSm5c2Oh/view?usp=sharing \ 

Drift Build \

colcon build   --packages-select tidybot_robot   --build-base drift/drift_build   --install-base drift/drift_install \
source drift/drift_install/setup.bash \
ros2 launch tidybot_robot tidybot_robot.launch.py \
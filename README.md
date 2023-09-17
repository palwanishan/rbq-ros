# rbq-ros
RBQ &lt;--> ROS api

## This branch is tested with ROS2-foxy

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b foxy https://github.com/palwanishan/rbq-ros.git

cd ~/ros2_ws && colcon build --packages-select rbq_ros

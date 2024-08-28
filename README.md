##### orbbec:git config --global http.postBuffer 1048576000

export ROS_PARALLEL_JOBS=-jn

ros-noetic-camera-infosss ros-noetic-dwa-local-planner
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
cd scripts
sudo cp 99-obsensor-ros1-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload && sudo udevadm trigger

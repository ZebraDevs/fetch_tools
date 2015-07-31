mkdir -p $FETCH_WORKSPACE/src &&
cd $FETCH_WORKSPACE/src &&
source /opt/ros/$ROS_DISTRO/setup.bash &&
catkin_init_workspace &&
cd .. &&
catkin_make;
echo "`sudo rosdep init` should fail unless you are using a brand new robot";
sudo rosdep init;  # Unnecessary on most robots
rosdep update;

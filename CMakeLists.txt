cmake_minimum_required(VERSION 2.8.3)
project(mushroom_ros)
find_package(catkin REQUIRED COMPONENTS rospy std_srvs)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/turtlebot_node.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



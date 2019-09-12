cmake_minimum_required(VERSION 2.8.3)
project(mushroom_ros)
find_package(catkin REQUIRED COMPONENTS rospy std_srvs)
catkin_package()

catkin_python_setup()

catkin_install_python(PROGRAMS nodes/turtlebot_node
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".git" EXCLUDE)



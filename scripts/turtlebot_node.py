#!/usr/bin/env python

from mushroom_ros.environments import TurtlebotGazebo
import rospy

rospy.init_node('turtlebot_node')
rospy.loginfo('Started turtlebot_node')
mdp = TurtlebotGazebo()
rospy.spin()

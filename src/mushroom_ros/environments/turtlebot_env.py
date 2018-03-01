import numpy as np
import tf
import rospy

from mushroom_ros import GazeboEnvironment
from mushroom.environments import MDPInfo
from mushroom.utils import spaces

from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates


class TurtlebotGazebo(GazeboEnvironment):
    def __init__(self, **kwargs):

        # Define environment properties
        high_x = np.array([5.0, 5.0, np.pi])
        low_x = -high_x

        high_u = np.array([1.0, 3.0])
        low_u = -high_u

        observation_space = spaces.Box(low=low_x, high=high_x)
        action_space = spaces.Box(low=low_u, high=high_u)

        gamma = 0.9
        horizon = 2000

        mdp_info = MDPInfo(observation_space, action_space, gamma, horizon)

        hz = 50.0

        super(TurtlebotGazebo, self).__init__('turtlebot_gazebo', mdp_info, hz, **kwargs)

        # subscribe to /cmd_vel topic to publish the setpoint
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # subscribe to /gazebo/model_states to get the position of the turtlebot
        self._sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self._state_callback)
        self._pose = None

    def _state_callback(self, msg):
        try:
            index = msg.name.index('turtlebot3_burger')
            self._pose = msg.pose[index]
            self._state_ready = True
        except ValueError:
            pass

    def publish_action(self, action):
        msg = Twist()

        msg.linear.x = action[0]
        msg.angular.z = action[1]

        self._pub.publish(msg)

    def get_state(self):
        x = self._pose.position.x
        y = self._pose.position.y

        quaternion = (
            self._pose.orientation.x,
            self._pose.orientation.y,
            self._pose.orientation.z,
            self._pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        yaw = euler[2]

        self._state_ready = False

        return np.array([x, y, yaw]), False

    def get_reward(self, state, action, next_state):
        target = np.array([2.0, 0.0, 0.0])
        return -(np.linalg.norm(target-next_state)+np.linalg.norm(action))


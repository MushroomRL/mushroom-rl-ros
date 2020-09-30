import numpy as np
import tf
import rospy

from mushroom_rl_ros import GazeboEnvironment
from mushroom_rl.environments import MDPInfo
from mushroom_rl.utils import spaces
from mushroom_rl.utils.angles import normalize_angle

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState

from tqdm import tqdm
tqdm.monitor_interval = 0


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
        horizon = 400

        mdp_info = MDPInfo(observation_space, action_space, gamma, horizon)

        hz = 10.0

        super(TurtlebotGazebo, self).__init__('turtlebot_gazebo', mdp_info, hz, **kwargs)

        # subscribe to /cmd_vel topic to publish the setpoint
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # subscribe to /gazebo/model_states to get the position of the turtlebot
        model_state_service_name = '/gazebo/get_model_state'
        rospy.wait_for_service(model_state_service_name)
        self._model_state_service = rospy.ServiceProxy(model_state_service_name, GetModelState)

    def publish_action(self, action):
        msg = Twist()

        msg.linear.x = action[0]
        msg.angular.z = action[1]

        self._pub.publish(msg)

    def get_state(self):
        ok = False
        while not ok:
            res = self._model_state_service('turtlebot3_burger', '')
            ok = res.success

        x = res.pose.position.x
        y = res.pose.position.y

        quaternion = (
            res.pose.orientation.x,
            res.pose.orientation.y,
            res.pose.orientation.z,
            res.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)

        yaw = normalize_angle(euler[2])

        return np.array([x, y, yaw]), False

    def get_reward(self, state, action, next_state, absorbing):
        target = np.array([2.0, 0.0, 0.0])
        return -np.linalg.norm(target-next_state)


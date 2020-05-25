import numpy as np
import rospy

from mushroom_rl_ros import ROSEnvironment
from mushroom_rl.environments import MDPInfo
from mushroom_rl.utils import spaces

from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from turtlesim.msg import Pose


from tqdm import tqdm
tqdm.monitor_interval = 0


class TurtleSim(ROSEnvironment):
    def __init__(self, **kwargs):

        # Define environment properties
        high_x = np.array([11.088889122009277, 11.088889122009277, np.pi])
        low_x = np.array([0, 0, -np.pi])

        high_u = np.array([2.0, 2.0])
        low_u = np.array([0., -2.0])

        observation_space = spaces.Box(low=low_x, high=high_x)
        action_space = spaces.Box(low=low_u, high=high_u)

        gamma = 0.9
        horizon = 100

        mdp_info = MDPInfo(observation_space, action_space, gamma, horizon)

        hz = 10.0

        self._target = np.array([2.0, 2.0, 0.0])
        self._current_pose = np.zeros(3)

        super().__init__('turtlesim_env', mdp_info, hz, **kwargs)

        # subscribe to /turtle1/cmd_vel topic to publish the setpoint
        self._cmd = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)

        # subscribe to /turtle1/pose to get the turtle pose
        self._pose = rospy.Subscriber('/turtle1/pose', Pose, self._pose_callback)

        # subscribe to the teleporting service to reset environment
        self._reset_service_name = '/turtle1/teleport_absolute'
        rospy.wait_for_service(self._reset_service_name)
        self._reset_service = rospy.ServiceProxy(self._reset_service_name, TeleportAbsolute)

    def start(self):
        initial_state = np.random.rand(3)
        initial_state *= self.info.observation_space.high
        rospy.wait_for_service(self._reset_service_name)
        self._reset_service(x=initial_state[0], y=initial_state[1], theta=initial_state[2])

    def stop(self):
        pass

    def publish_action(self, action):
        action = self._bound(action, self.info.action_space.low, self.info.action_space.high)

        msg = Twist()

        msg.linear.x = action[0]
        msg.angular.z = action[1]

        self._cmd.publish(msg)

    def get_reward(self, state, action, next_state, absorbing):
        return -np.linalg.norm(self._target - next_state)

    def get_state(self):
        state = self._current_pose.copy()
        goal_reached = np.linalg.norm(self._target - state) < 0.1

        return state, goal_reached

    def _pose_callback(self, msg):
        self._current_pose[0] = msg.x
        self._current_pose[0] = msg.y
        self._current_pose[0] = msg.theta

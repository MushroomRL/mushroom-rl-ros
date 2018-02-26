import rospy
from mushroom.environments import Environment


class ROSEnvironment(Environment):
    def __init__(self, name, mdp_info, hz, **kwargs):
        rospy.init_node(name, **kwargs)

        self._r = rospy.Rate(hz)
        self._state_ready = False
        self._state = None

        self.__name__ = name
        super(ROSEnvironment, self).__init__(mdp_info)

    def reset(self, state=None):
        self.start()

        while not self._state_ready and not rospy.is_shutdown():
            self._r.sleep()

        self._state = self.get_state()
        self._r.reset()

        return self._state

    def step(self, action):
        while not self._state_ready and not rospy.is_shutdown():
            self.publish_action(action)
            self.r.sleep()

        next_state, absorbing = self.get_state()
        reward = self.get_reward(self._state, action, next_state)

        self._state = next_state

        if absorbing:
            self.stop()

        return self._state, reward, absorbing, {}

    def render(self, mode='human', close=False):
        pass

    def start(self):
        """
        This method contains the logic needed to start the specific environment.
        Must be implemented.

        """
        raise NotImplementedError

    def stop(self):
        """
        This method contains the logic needed to stop the specific environment.
        Must be implemented.

        """
        raise NotImplementedError

    def publish_action(self, action):
        """
        This method contains the logic used to publish actions to the system,
        e.g., publish a velocity setpoint on the /cmd_vel topic.
        Must be implemented.

        Args:
            action (np.ndarray): the action selected by the controller.

        """
        raise NotImplementedError

    def get_state(self):
        """
        This method contains the logic to get the current state.
        Must be implemented.

        Returns:
            The current state of the environment (e.g. the robot state).

        """
        raise NotImplementedError

    def get_reward(self, state, action, next_state):
        """
        This method contains the reward function implementation.
        Must be implemented.

        Args:
            state (np.ndarray): the previous state.
            action (np.ndarray): the action taken.
            next_state (np.ndarray): the state reached.

        Returns:
            The value of the reward for the specified transition.

        """
        raise NotImplementedError
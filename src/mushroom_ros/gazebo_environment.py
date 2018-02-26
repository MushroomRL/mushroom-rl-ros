import rospy
from ros_environment import ROSEnvironment
from std_srvs.srv import Empty


class GazeboEnvironment(ROSEnvironment):
    def __init__(self, name, mdp_info, hz, **kwargs):
        super(GazeboEnvironment, self).__init__(name, mdp_info, hz, **kwargs)

        self._simulation_running = True

        rospy.loginfo('Waiting for Gazebo')

        reset_service_name = '/gazebo/reset_simulation'
        reset_world_service_name = '/gazebo/reset_world'
        pause_service_name = '/gazebo/pause_physics'
        resume_service_name = '/gazebo/unpause_physics'

        # Wait for gazebo services
        rospy.wait_for_service(reset_service_name)
        rospy.wait_for_service(reset_world_service_name)
        rospy.wait_for_service(pause_service_name)
        rospy.wait_for_service(resume_service_name)

        # Create server proxies
        self._reset_service = rospy.ServiceProxy(reset_service_name, Empty)
        self._reset_world_service = rospy.ServiceProxy(reset_world_service_name, Empty)
        self._pause_service = rospy.ServiceProxy(pause_service_name, Empty)
        self._resume_service = rospy.ServiceProxy(resume_service_name, Empty)

        # Stop simulation
        rospy.loginfo('Gazebo simulator is up, stopping simulation')
        self.stop()

    def start(self):
        if self._simulation_running:
            self.stop()

        self._resume_service()
        self._simulation_running = True

    def stop(self):
        self._pause_service()
        self._reset_service()
        self._simulation_running = False

    '''def _on_exit(self):
        if self._simulation_running:
            rospy.loginfo('Simulation is still running, stopping...')
            self.stop()'''
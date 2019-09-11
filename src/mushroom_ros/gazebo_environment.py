import rospy
from ros_environment import ROSEnvironment
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState


class GazeboEnvironment(ROSEnvironment):
    def __init__(self, name, mdp_info, hz, **kwargs):
        super(GazeboEnvironment, self).__init__(name, mdp_info, hz, **kwargs)

        self._simulation_running = True

        rospy.loginfo('Waiting for Gazebo')

        reset_service_name = '/gazebo/reset_simulation'
        reset_world_service_name = '/gazebo/reset_world'
        pause_service_name = '/gazebo/pause_physics'
        resume_service_name = '/gazebo/unpause_physics'
        get_model_service_name = '/gazebo/get_model_state'
        set_model_service_name = '/gazebo/set_model_state'
        get_link_service_name = '/gazebo/get_link_state'
        set_link_service_name = '/gazebo/set_link_state'

        # Wait for gazebo services
        rospy.wait_for_service(reset_service_name)
        rospy.wait_for_service(reset_world_service_name)
        rospy.wait_for_service(pause_service_name)
        rospy.wait_for_service(resume_service_name)
        rospy.wait_for_service(set_model_service_name)
        rospy.wait_for_service(get_model_service_name)
        rospy.wait_for_service(set_link_service_name)
        rospy.wait_for_service(get_link_service_name)

        # Create server proxies
        self._reset_service = rospy.ServiceProxy(reset_service_name, Empty)
        self._reset_world_service = rospy.ServiceProxy(reset_world_service_name, Empty)
        self._pause_service = rospy.ServiceProxy(pause_service_name, Empty)
        self._resume_service = rospy.ServiceProxy(resume_service_name, Empty)
        self._get_model_service = rospy.ServiceProxy(get_model_service_name, GetModelState)
        self._set_model_service = rospy.ServiceProxy(set_model_service_name, SetModelState)

        # Stop simulation
        rospy.loginfo('Gazebo simulator is up')
        self.stop()

    def start(self):
        if self._simulation_running:
            self.stop()

        self._resume_service()
        self.start_extra()
        self._simulation_running = True

    def stop(self):
        self.stop_extra()
        self._pause_service()
        self._reset_service()
        self._simulation_running = False

    def start_extra(self):
        pass

    def stop_extra(self):
        pass

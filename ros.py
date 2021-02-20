import numpy as np
import os
import subprocess
import sys
import rospy
import op3
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty

class RosController:
    def __init__(self, launchfile):
        # self.last_clock_msg = Clock()
        ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))
        self._roslaunch = subprocess.Popen([
            sys.executable,
            os.path.join(ros_path, b"roslaunch"),
            launchfile
        ])
        rospy.init_node('gym', anonymous=True)

        self.latest_link_states = None

        def link_states_cb(data):
            self.latest_link_states = data

        self.publishers = [rospy.Publisher(topic, Float64, queue_size=20) for topic in op3.command_topics]
        self.link_states_subscriber = rospy.Subscriber('/gazebo/link_states', LinkStates, link_states_cb, queue_size=10)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.listControllers = rospy.ServiceProxy('/robotis_op3/controller_manager/list_controllers', ListControllers)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.prev_action = np.zeros(20)

    def get_link_states(self):
        return self.latest_link_states
    
    def publish_action(self, action):
        for index, pub in enumerate(self.publishers):
            value = Float64()
            value.data = action[index]
            pub.publish(value)
    
    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")
    
    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
    
    def reset(self):
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")
    
    def wait_for_controllers(self):
        rospy.wait_for_service('/robotis_op3/controller_manager/list_controllers')
        ctrls = None
        while True:
            ctrls = self.listControllers().controller
            ctrl_names = [x.name for x in ctrls]
            flag = True
            for name in op3.controller_names:
                if name not in ctrl_names:
                    flag = False
                    break
            if flag: break

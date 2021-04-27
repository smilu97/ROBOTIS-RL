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

class RosController(object):
    def __init__(self, launchfile):
        # self.last_clock_msg = Clock()

        if False:
            random_number = random.randint(10000, 15000)
            # self.port = "11311"#str(random_number) #os.environ["ROS_PORT_SIM"]
            # self.port_gazebo = "11345"#str(random_number+1) #os.environ["ROS_PORT_SIM"]
            self.port = str(random_number) #os.environ["ROS_PORT_SIM"]
            self.port_gazebo = str(random_number+1) #os.environ["ROS_PORT_SIM"]

            os.environ["ROS_MASTER_URI"] = "http://localhost:"+self.port
            os.environ["GAZEBO_MASTER_URI"] = "http://localhost:"+self.port_gazebo
            #
            # self.ros_master_uri = os.environ["ROS_MASTER_URI"];

            print("ROS_MASTER_URI=http://localhost:"+self.port + "\n")
            print("GAZEBO_MASTER_URI=http://localhost:"+self.port_gazebo + "\n")

        ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))
        self._roslaunch = subprocess.Popen([
            sys.executable,
            os.path.join(ros_path, b"roslaunch"),
            launchfile
        ])
        rospy.init_node('gym', anonymous=True)

        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_sim_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    
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
    
    def reset_world(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_world service call failed")
    
    def reset_sim(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_sim_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

    def reset(self):
        # self.pause()
        self.reset_sim()
        self.reset_world()

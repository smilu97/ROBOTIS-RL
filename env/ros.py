import numpy as np
import os
import random
import subprocess
import sys
import rospy
import rospkg
import time

from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import DeleteModel, SpawnModel
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty

class RosController(object):
    def __init__(self, launchfile, randomize_port=True):
        self.launchfile = launchfile
        self.randomize_port = randomize_port
        self.create()

        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.reset_sim_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        self.spawn_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

    def __del__(self):
        self.destroy()
    
    def create(self):
        # self.last_clock_msg = Clock()

        if self.randomize_port:
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
            self.launchfile
        ])
        rospy.init_node('gym' + str(random.randint(1000, 1500)), anonymous=True)

    def render(self, mode="human", close=False):
        if close:
            tmp = os.popen("ps -Af").read()
            proccount = tmp.count('gzclient')
            if proccount > 0:
                if self.gzclient_pid != 0:
                    os.kill(self.gzclient_pid, signal.SIGTERM)
                    os.wait()
            return

        tmp = os.popen("ps -Af").read()
        proccount = tmp.count('gzclient')
        if proccount < 1:
            subprocess.Popen("gzclient")
            self.gzclient_pid = int(subprocess.check_output(["pidof","-s","gzclient"]))
        else:
            self.gzclient_pid = 0

    def destroy(self):
        rospy.signal_shutdown('fin')
        for name in ['gzserver', 'gzclient', 'roscore', 'rosmaster']:
            try:
                subprocess.check_output(['killall', '-9', name])
            except:
                pass

    def unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")
    
    def pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
    
    def reset_world(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_world service call failed")
    
    def reset_sim(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_sim_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")
    
    def delete_model(self):
        rospy.wait_for_service('/gazebo/delete_model')
        try:
            self.delete_model_proxy('robotis_op3')
        except (rospy.ServiceException) as e:
            print ("/gazebo/delete_model service call failed")
    
    def spawn_model(self):
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            model_xml = rospy.get_param('robot_description')
            initial_pose = Pose()
            initial_pose.position.x = 0
            initial_pose.position.y = 0
            initial_pose.position.z = 0.285
            self.spawn_model_proxy('robotis_op3', model_xml, '', initial_pose, 'world')
        except (rospy.ServiceException) as e:
            print ("/gazebo/spawn_urdf_model service call failed")

    def reset(self):
        # self.pause()
        self.reset_sim()
        self.reset_world()

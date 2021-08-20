import numpy as np
import os
import random
import subprocess
import sys
import rospy

from .proxy.unpause import UnpauseProxy
from .proxy.pause import PauseProxy
from .proxy.reset_world import ResetWorldProxy
from .proxy.reset_simulation import ResetSimulationProxy
from .proxy.delete_model import DeleteModelProxy
from .proxy.iterate import IterateProxy
from .proxy.spawn_op3 import SpawnOp3Proxy

class RosController(object):
    def __init__(self, launchfile, randomize_port=True):
        if launchfile is None:
            launchfile = os.path.dirname(os.path.abspath(__file__)) + '/op3.launch'
        self.launchfile = launchfile
        self.randomize_port = randomize_port
        self.create()

        self.unpause = UnpauseProxy()
        self.pause = PauseProxy()
        self.reset_world = ResetWorldProxy()
        self.reset_sim = ResetSimulationProxy()
        self.iterate = IterateProxy()
        self.spawn_model = SpawnOp3Proxy()
        self.delete_model = DeleteModelProxy('robotis_op3')

    def __del__(self):
        self.destroy()
    
    def reset(self):
        self.iterate.reset() # iterate is stateful
    
    def create(self):
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

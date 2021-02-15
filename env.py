import numpy as np
import os
import subprocess
import sys
import rospy
import op3

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from gazebo_msgs.msg import LinkStates
from controller_manager_msgs.srv import ListControllers
from std_srvs.srv import Empty

class Environment:
    def __init__(self, launchfile):
        # self.last_clock_msg = Clock()
        ros_path = os.path.dirname(subprocess.check_output(["which", "roscore"]))
        self._roslaunch = subprocess.Popen([
            sys.executable,
            os.path.join(ros_path, b"roslaunch"),
            launchfile
        ])
        rospy.init_node('gym', anonymous=True)

        self.publishers = [rospy.Publisher(topic, Float64, queue_size=2) for topic in op3.command_topics]
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.listControllers = rospy.ServiceProxy('/robotis_op3/controller_manager/list_controllers', ListControllers)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.prev_action = np.zeros(20)

    def scan_data(self):
        while True:
            try:
                data = rospy.wait_for_message("/gazebo/link_states", LinkStates, timeout=5)
                if data is not None:
                    return data
            except:
                pass
    
    def supress_action(self, action):
        if self.prev_action is None:
            return action
        prev = np.array(self.prev_action)
        curr = np.array(action)
        delta = curr - prev

        max_speed = 0.05
        delta = np.minimum(delta, max_speed)
        delta = np.maximum(delta, -max_speed)
        supressed_action = prev + delta

        self.prev_action = supressed_action

        return supressed_action
    
    def calculate_observation(self, data):
        names = data.name
        # - robotis_op3::body_link
        # - robotis_op3::head_pan_link
        # - robotis_op3::head_tilt_link
        for index, name in enumerate(data.name):
            if name != "robotis_op3::body_link": continue
            p = data.pose[index].position
            return (p.x, p.y, p.z), False
        return None, False
    
    def _unpause(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")
    
    def _pause(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")
    
    def _reset(self):
        # Resets the state of the environment and returns an initial observation.
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")
        
    def step(self, action):
        self._unpause()

        action = self.supress_action(action)
        for index, pub in enumerate(self.publishers):
            value = Float64()
            value.data = action[index]
            pub.publish(value)

        data = self.scan_data()
        self._pause()
        state, done = self.calculate_observation(data)
        reward = 0
        return np.asarray(state), reward, done, {}
    
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
        print 'Checked all controllers available'
        for ctrl in ctrls:
            print ctrl.name, ctrl.state

    def reset(self):
        self.wait_for_controllers()
        self._reset()
        self._unpause()
        data = self.scan_data()
        self._pause()
        state,done = self.calculate_observation(data)
        return np.asarray(state)
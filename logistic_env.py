import numpy as np

from env import Op3Environment
from op3constant import op3_module_names as oms

class Op3LogisticEnvrionment(Op3Environment):
    def __init__(self, launchfile, speed=0.03):
        super(Op3LogisticEnvrionment, self).__init__(launchfile)
        self.joint_goals = np.zeros(len(oms))
        self.speed = speed
        self.max_joint =  0.7
        self.min_joint = -0.7
    
    def reset(self):
        self.joint_goals = np.zeros(len(oms))
        return super(Op3LogisticEnvrionment, self).reset()

    def apply_action(self, action):
        self.joint_goals = np.minimum(self.max_joint, np.maximum(self.min_joint, self.joint_goals + ((action - 2) * self.speed)))
        self.op3.publish_action2(self.joint_goals)
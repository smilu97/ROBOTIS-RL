from . import RosProxy

from op3_gym.srv import Step

class UnpauseProxy(RosProxy):
    def __init__(self):
        super().__init__('/gazebo/unpause_physics', Step)

from . import RosProxy

class PauseProxy(RosProxy):
    def __init__(self):
        super().__init__('/gazebo/pause_physics')

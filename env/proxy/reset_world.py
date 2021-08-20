from . import RosProxy

class ResetWorldProxy(RosProxy):
    def __init__(self):
        super().__init__('/gazebo/reset_world')

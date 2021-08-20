from . import RosProxy

class ResetSimulationProxy(RosProxy):
    def __init__(self):
        super().__init__('/gazebo/reset_simulation')

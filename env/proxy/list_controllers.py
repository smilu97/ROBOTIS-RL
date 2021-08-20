from . import RosProxy

from controller_manager_msgs.srv import ListControllers

class ListControllersProxy(RosProxy):
    def __init__(self):
        super().__init__('/robotis_op3/controller_manager/list_controllers', ListControllers)

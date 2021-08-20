from . import RosProxy

from gazebo_msgs.srv import DeleteModel

class DeleteModelProxy(RosProxy):
    def __init__(self, model):
        self.model = model
        super().__init__('/gazebo/delete_model', DeleteModel)

    def __call__(self):
        super().__call__(self.model)

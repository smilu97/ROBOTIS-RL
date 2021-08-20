from . import RosSubscriber

from gazebo_msgs.msg import LinkStates

class LinkStatesSubscriber(RosSubscriber):
    def __init__(self):
        super().__init__('/gazebo/link_states', LinkStates, 10)

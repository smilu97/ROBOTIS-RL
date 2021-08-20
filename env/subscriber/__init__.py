import rospy

class RosSubscriber:
    def __init__(self, path, state_type, queue_size = 10):
        self.updated = False
        self.latest = None

        def callback(data):
            self.updated = True
            self.latest = data

        self.subscriber = \
            rospy.Subscriber(path, state_type, callback, queue_size=queue_size)
    
    def reset(self):
        self.updated = False
        self.latest = None

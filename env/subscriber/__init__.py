import rospy
import threading

class RosSubscriber:
    def __init__(self, path, state_type, queue_size = 10):
        self.updated = False
        self.latest = None
        self.wakeup = False
        self.evt_wakeup = threading.Event()

        def callback(data):
            self.updated = True
            self.latest = data
            self.wakeup = True
            self.evt_wakeup.set()

        self.subscriber = \
            rospy.Subscriber(path, state_type, callback, queue_size=queue_size)
    
    def wait(self):
        if self.wakeup: return
        self.evt_wakeup.wait(0.1)
        self.wakeup = False
        self.evt_wakeup.clear()

    def reset(self):
        self.updated = False
        self.latest = None

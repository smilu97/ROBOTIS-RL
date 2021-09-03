import rospy
import threading

from . import RosProxy
from rosgraph_msgs.msg import Clock
from op3_gym.srv import Step, StepRequest

def clock_to_int(c):
    c = c.clock
    return c.secs * 1000000000 + c.nsecs

class IterateProxy(RosProxy):
    def __init__(self):
        super().__init__('/iterate', Step)

        self.last_clock = 0
        self.target_clock = 0
        self.clock_event = threading.Event()

        def clock_cb(data):
            self.last_clock = clock_to_int(data)
            if self.last_clock >= self.target_clock:
                self.clock_event.set()
            # print('last: {}, target: {}, set: {}'.format(self.last_clock, self.target_clock, self.last_clock >= self.target_clock))
            
        self.clock_subscriber = rospy.Subscriber('/clock', Clock, clock_cb, queue_size=10)
    
    def reset(self):
        self.last_clock = 0
        self.target_clock = 0
    
    def __call__(self, n):
        self.target_clock = self.last_clock + n * 1000000
        req = StepRequest()
        req.iterations = n
        self.clock_event.clear()
        super().__call__(req)
        if self.last_clock < self.target_clock:
            self.clock_event.wait(0.5)

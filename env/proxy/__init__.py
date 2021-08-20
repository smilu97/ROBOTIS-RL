import rospy

from std_srvs.srv import Empty

class RosProxy:
    def __init__(self, path, payload = Empty):
        self.path = path 
        self.payload = payload

        self.proxy = rospy.ServiceProxy(path, payload)
    
    def __call__(self, *args, **kwargs):
        rospy.wait_for_service(self.path)
        try:
            return self.proxy(*args, **kwargs)
        except (rospy.ServiceException) as e:
            print("{} service call failed".format(self.path))
            raise e

import numpy as np

from . import Middleware

class OrthogonalKnee(Middleware):
    
    def __call__(self, payload, context):
        ori = self.env.op3.imu.latest.orientation

        result = np.array(payload)

        y = ori.y * 1.6
        result[7 ] -= payload[4 ] + y
        result[16] -= payload[13] - y

        return result

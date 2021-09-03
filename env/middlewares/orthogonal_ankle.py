import numpy as np

from . import Middleware

def state_to_ank(state, left=True):
    x, y = state[0].x, state[0].y
    if not left:
        x *= -1
        y *= -1
    x *= 0.9
    y *= 0.9
    return state[1] + state[3] + y, state[2] - x

class OrthogonalAnkle(Middleware):

    def __call__(self, payload, context):
        ori = self.env.op3.imu.latest.orientation

        # ori, hip_pitch, hip_roll, knee
        l_state = (ori, payload[4], payload[5], payload[7])
        r_state = (ori, payload[13], payload[14], payload[16])

        result = np.array(payload)

        l_ank_pitch, l_ank_roll = state_to_ank(l_state)
        result[1] += l_ank_pitch
        result[2] += l_ank_roll

        r_ank_pitch, r_ank_roll = state_to_ank(r_state, left=False)
        result[10] += r_ank_pitch
        result[11] += r_ank_roll

        return result

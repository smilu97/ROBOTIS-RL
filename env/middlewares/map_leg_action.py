import numpy as np

from . import Middleware

class MapLegAction(Middleware): 
    '''
    다리 관절들만 움직이는 action이 들어온 것을 전체 관절들에 매핑
    '''

    def __call__(self, payload, context):
        return np.array([
            0, # head_tilt 0
            payload[0],  # l_ank_pitch 1
            payload[1],  # l_ank_roll 2
            0,  # l_el 3
            payload[2],  # l_hip_pitch 4
            payload[3],  # l_hip_roll 5
            payload[4],  # l_hip_yaw 6
            payload[5],  # l_knee 7
            0,  # l_sho_pitch 8
            0,  # l_sho_roll 9
            payload[6],  # r_ank_pitch 10
            payload[7], # r_ank_roll 11
            0, # r_el 12
            payload[8], # r_hip_pitch 13
            payload[9], # r_hip_roll 14
            payload[10], # r_hip_yaw 15
            payload[11], # r_knee 16
            0, # r_sho_pitch 17
            0, # r_sho_roll 18
        ], dtype=np.float32)

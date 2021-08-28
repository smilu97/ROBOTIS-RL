import numpy as np

from . import Middleware

def unsym_sin(x, w=0.35):
    r = np.sin(x)
    if r < 0:
        r *= w
    return r

class HumanBiasAction(Middleware):
    '''
    임의로 사람이 정한 행동을 open loop 하게 출력
    '''

    def __init__(self, env, T, multiplier=0.5):
        super().__init__(env)
        self.T = T
        self.multiplier = multiplier

    def get_bias(self, t):
        m = self.multiplier

        ank_pitch = 30.0 * m
        hip_pitch = 60.0 * m
        hip_roll  = 10.0 * m
        knee      = 45.0 * m
        sho_pitch = 45.0 * m

        return np.array([
            0, # head_tilt 0
            -ank_pitch * unsym_sin(t, -0.35), # l_ank_pitch 1
            0, # l_ank_roll 2
            0, # l_el 3
            -hip_pitch * unsym_sin(t), # l_hip_pitch 4
            hip_roll * unsym_sin(t, 1), # l_hip_roll 5
            0, # l_hip_yaw 6
            knee * unsym_sin(t, 0), # l_knee 7
            sho_pitch * unsym_sin(t, 1), # l_sho_pitch 8
            0, # l_sho_roll 9
            ank_pitch * unsym_sin(t+np.pi, -0.35), # r_ank_pitch 10
            0, # r_ank_roll 11
            0, # r_el 12
            hip_pitch * unsym_sin(t+np.pi), # r_hip_pitch 13
            hip_roll * unsym_sin(t, 1), # r_hip_roll 14
            0, # r_hip_yaw 15
            -knee * unsym_sin(t+np.pi, 0), # r_knee 16
            -sho_pitch * unsym_sin(t+np.pi, 1), # r_sho_pitch 17
            0, # r_sho_roll 18
        ], dtype=np.float32) / 180 * np.pi * 0.5

    def get_human_bias_reference(self):
        t = 2 * np.pi * self.env.t / self.T
        return self.get_bias(t)

    def __call__(self, payload, context):
        return payload + self.get_human_bias_reference()

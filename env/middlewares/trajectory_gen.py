import numpy as np

from . import Middleware

class TrajectoryGenerator(Middleware):
    '''
    introduced in http://proceedings.mlr.press/v100/yang20a/yang20a.pdf
    '''
    def __init__(self, env, length):
        super().__init__(env)
        self.length = length
        self.pi = np.zeros(length)
    
    def reset(self):
        self.pi = np.zeros(self.length)
    
    def generate(self, pi_add, ce, a_stance, a_lift):
        self.pi += pi_add
        self.pi -= np.floor(self.pi / (2 * np.pi)) * (2 * np.pi)

        pi_stance = np.pi

        stances = self.pi <  pi_stance
        lifts   = self.pi >= pi_stance
        
        stance_pi = (self.pi / pi_stance) * np.pi
        lift_pi   = (1 + ((self.pi - pi_stance) / ((2 * np.pi) - pi_stance))) * np.pi

        return ce + \
            (stances * a_stance * np.sin(stance_pi)) + \
            (lifts   * a_lift   * np.sin(lift_pi  ))

    def __call__(self, payload, context):
        n = self.length

        pi_add = (payload[:n] + 1) / 2
        ce = payload[n:2*n] / 10
        a_stance = payload[2*n:3*n]
        a_lift   = payload[3*n:4*n]

        return self.generate(pi_add, ce, a_stance, a_lift)

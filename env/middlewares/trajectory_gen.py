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
        self.t = 0
    
    def reset(self):
        self.pi = np.zeros(self.length)
        self.t = 0
    
    def generate(self, pi_add, ce):
        self.pi += pi_add
        self.pi -= np.floor(self.pi / (2 * np.pi)) * (2 * np.pi)

        pi_stance = np.pi

        stances = self.pi <  pi_stance
        lifts   = self.pi >= pi_stance
        a_stance = 1.0
        a_lift = 1.0
        
        stance_pi = (self.pi / pi_stance) * np.pi
        lift_pi   = (1 + ((self.pi - pi_stance) / ((2 * np.pi) - pi_stance))) * np.pi

        return ce + \
            (stances * a_stance * np.sin(stance_pi)) + \
            (lifts   * a_lift   * np.sin(lift_pi  ))

    def __call__(self, payload, context):
        n = self.length
        
        dt = self.env.t - self.t
        self.t = self.env.t

        pi_add = payload[:n] * dt * 30
        ce = payload[n:2*n] * 0.3

        return self.generate(pi_add, ce)

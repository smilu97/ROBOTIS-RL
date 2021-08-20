import numpy as np

from . import Middleware

class TrajectoryGenerator(Middleware):
    '''
    introduced in http://proceedings.mlr.press/v100/yang20a/yang20a.pdf
    '''
    def __init__(self, env, length, ce, a_stance, a_lift, pi_stance):
        super().__init__(env)
        self.ce = ce
        self.length = length
        self.a_stance = a_stance
        self.a_lift = a_lift
        self.pi_stance = pi_stance

        self.pi = np.zeros(len(pi_stance))
    
    def generate(self, payload):
        self.pi += payload
        self.pi -= np.floor(self.pi / (2 * np.pi)) * (2 * np.pi)

        stances = self.pi < self.pi_stance
        lifts = self.pi >= self.pi_stance
        
        stance_pi = (self.pi / self.pi_stance) * np.pi
        lift_pi = (1 + ((self.pi - self.pi_stance) / ((2 * np.pi) - self.pi_stance))) * np.pi

        return self.ce + \
            (stances * self.a_stance * np.sin(stance_pi)) + \
            (lifts   * self.a_lift   * np.sin(lift_pi  ))

    def __call__(self, payload, context):
        return payload[self.length:] + self.generate(payload[:self.length])

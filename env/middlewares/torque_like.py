import numpy as np

from . import Middleware

class TorqueLike(Middleware):
    '''
    introduced in http://proceedings.mlr.press/v100/yang20a/yang20a.pdf
    '''
    def __init__(self, env, length):
        super().__init__(env)
        self.length = length
        self.memory = np.zeros(length)
    
    def reset(self):
        self.memory = np.zeros(self.length)

    def __call__(self, payload, context):
        self.memory += payload
        return self.memory

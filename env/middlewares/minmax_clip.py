import numpy as np

from . import Middleware

class MinMaxClip(Middleware):
    
    def __init__(self, env, min_value=-1.0, max_value=1.0):
        super().__init__(env)
        self.min = min_value
        self.max = max_value
    
    def __call__(self, payload, context):
        return np.minimum(self.max, np.maximum(self.min, payload))

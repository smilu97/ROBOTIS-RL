import numpy as np

from . import Middleware

class ExponentialMove(Middleware):
    '''
    새로운 입력이 들어와도 이전 입력이 일정 비율 살아남도록
    '''

    def __init__(self, env, rate):
        super().__init__(env)
        self.rate = rate
        self.latest = None
    
    def reset(self):
        self.latest = None
    
    def __call__(self, payload, context):
        if self.latest is None:
            self.latest = payload
        else:
            rate_ = 1 - self.rate
            self.latest = (self.latest * self.rate) + (payload * rate_)

        return self.latest
        
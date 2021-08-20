from . import Middleware

class Multiply(Middleware):

    def __init__(self, env, v):
        super().__init__(env)
        self.v = v
    
    def __call__(self, payload, context):
        return payload * self.v

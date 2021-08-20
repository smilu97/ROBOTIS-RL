class Middleware:
    def __init__(self, env):
        self.env = env

    def reset(self):
        pass

    def __call__(self, payload, context):
        raise NotImplementedError('__call__ is not implemented')

def apply_middlewares(payload, middlewares):
    context = dict()
    for middleware in middlewares:
        payload = middleware(payload, context)
    return payload

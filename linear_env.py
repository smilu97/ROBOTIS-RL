from env import Op3Environment

class Op3LinearEnvironment(Op3Environment):
    def __init__(self, launchfile):
        super(Op3LinearEnvironment, self).__init__(launchfile)

import gym

from .walker_base_env import WalkerBaseBulletEnv
from .humanoid import Humanoid

class HumanoidBulletEnv(WalkerBaseBulletEnv):
    def __init__(self, robot=None):
        self.robot = robot if robot is not None else Humanoid()
        WalkerBaseBulletEnv.__init__(self, self.robot)
        self.electricity_cost = 4.25 * WalkerBaseBulletEnv.electricity_cost
        self.stall_torque_cost = 4.25 * WalkerBaseBulletEnv.stall_torque_cost

gym.envs.register(
    id='RoboschoolHumanoid-v0',
    entry_point=HumanoidBulletEnv
)

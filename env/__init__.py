import numpy as np
import gym

from . import constants as op3c
from .op3 import Op3Controller, serialize_imu
from .params import params

from .middlewares import apply_middlewares
from .middlewares.add import Add
from .middlewares.multiply import Multiply
from .middlewares.exp_move import ExponentialMove
from .middlewares.map_leg_action import MapLegAction
from .middlewares.minmax_clip import MinMaxClip
from .middlewares.human_bias_action import HumanBiasAction
from .middlewares.trajectory_gen import TrajectoryGenerator
from .middlewares.orthogonal_ankle import OrthogonalAnkle
from .middlewares.orthogonal_knee import OrthogonalKnee

progress_bonus = params.progress_bonus
accel_bonus = params.accel_bonus
alive_bonus = params.alive_bonus
outroute_cost = params.outroute_cost
velocity_cost = params.velocity_cost
height_bonus = params.height_bonus
effort_cost = params.effort_cost
stuck_cost = params.stuck_cost
action_modify_rate = params.action_modify_rate
reference_cycle = params.reference_cycle
reference_weight = params.reference_weight

def unsym_sin(x, w=0.35):
    r = np.sin(x)
    if r < 0:
        r *= w
    return r

class OP3Env(gym.Env):
    def __init__(
        self,
        human_bias=False,
        step_size=0.02,
        random_port=True,
        print_rewards=False,
        use_bias=True
    ):
    
        self.print_rewards = print_rewards
        self.human_bias = human_bias
        self.step_size = step_size
        self.num_mod = len(op3c.op3_module_names)

        self.action_bias = np.array(op3c.joint_bias) / 180 * np.pi
        self.action_range = np.array(op3c.joint_ranges) / 180 * np.pi
        self.reward_debug = np.zeros(3)
        
        self.op3 = Op3Controller(random_port=random_port)
        self.reset_variables()

        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(48,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(34,), dtype=np.float32)

        self.action_middlewares = [
            TrajectoryGenerator(self, 12),
            MapLegAction(self),
            Multiply(self, self.action_range),
            Add(self, self.action_bias),
            HumanBiasAction(self, T=1.5) if human_bias else None,
            # ExponentialMove(self, params.action_modify_rate),
            # OrthogonalKnee(self),
            # OrthogonalAnkle(self),
            MinMaxClip(self, -1.5, 1.5)
        ]

        if not use_bias:
            self.action_bias[:] = 0
    
    def reset_variables(self):
        self.prev_x = 0.0
        self.prev_dx = 0.0
        self.prev_z = None
        self.t = 0.0

    def render(self, **kwargs):
        self.op3.render(**kwargs)
    
    def get_observation(self):
        self.op3.joint_states.wait()

        joint_states = self.op3.joint_states.latest
        joint_dict = {}
        if joint_states is not None:
            for index, name in enumerate(joint_states.name):
                joint_dict[name] = (
                    joint_states.position[index],
                    joint_states.velocity[index] * 0.01,
                    joint_states.effort[index] * 0.05,
                )
        self.positions  = np.array([joint_dict[name][0] for name in op3c.op3_obs_module_names], dtype=np.float32)
        # velocities = np.array([joint_dict[name][1] for name in op3c.op3_obs_module_names])
        self.efforts    = [joint_dict[name][2] for name in op3c.op3_obs_module_names]

        imu = serialize_imu(self.op3.imu.latest)

        t = self.t / 1.5
        t -= np.floor(t)

        tg_pi = self.action_middlewares[0].pi

        return np.concatenate([self.positions, tg_pi, imu])
    
    def count_stuck(self, obs):
        position = obs[:self.num_mod]
        p1 = self.action_bias - self.action_range
        p2 = self.action_bias + self.action_range
        return np.sum(position <= p1) + np.sum(position >= p2)

    def get_reward(self, obs, position, done):
        dx = position.x - self.prev_x
        ddx = dx - self.prev_dx
        # dz = 0 if self.prev_z is None else position.z - self.prev_z
        self.prev_x = position.x
        # self.prev_z = position.z
        self.prev_dx = dx

        # accel = ddx * accel_bonus
        progress = dx * progress_bonus
        # height = position.z *.height_bonus
        alive = -50.0 if done else alive_bonus
        # outroute = abs(position.y) * outroute_cost
        # velocity = np.sum(np.abs(obs[self.num_mod : 2*self.num_mod])) * velocity_cost
        effort = np.sum(np.abs(self.efforts)) * effort_cost
        # stuck = self.count_stuck(obs) * self.stuck_cost

        rewards = np.array((progress, alive, effort))
        self.reward_debug += rewards

        return np.sum(rewards)
    
    def get_done(self, position):
        return self.t >= 50.0 or position.z < 0.2
    
    def get_body_position(self):
        target = 'robotis_op3::body_link'
        link_states = self.op3.link_states.latest
        for index, name in enumerate(link_states.name):
            if name == target:
                return link_states.pose[index].position
        return 0.0

    def step(self, action):

        self.op3.act(apply_middlewares(action, self.action_middlewares))
        self.op3.iterate(25) # 200 Hz

        position = self.get_body_position()
        done = self.get_done(position)
        obs = self.get_observation()

        if np.any(np.isnan(obs)):
            raise Exception('robot broken down!')
            
        reward = self.get_reward(obs, position, done)
        self.t += 0.025

        if self.print_rewards and done:
            print('rewards:', self.reward_debug)
            self.reward_debug[:] = 0

        return (
            obs,
            reward,
            done,
            {}
        )

    def reset(self):
        self.reset_variables()
        self.op3.reset()

        for mid in self.action_middlewares:
            if mid is not None: mid.reset()

        return self.get_observation()
    
    def reset_op3(self):
        self.op3 = Op3Controller()

gym.envs.register(
    id='RobotisOp3-v0',
    entry_point=OP3Env
)

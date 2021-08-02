# author: smilu97

import gym
import ray
import numpy as np

from typing import Callable

from ars.config import ARSConfig
from ars.policy import Policy
from ars import ARS

@ray.remote
class EnvActor:
  def __init__(self, env_creator: Callable[[], gym.Env], policy: Policy):
    self.env = env_creator()
    self.policy = policy
  
  def evaluate_n_times(self, params: np.ndarray, n: int=1):
    rewards = []
    for _ in range(n):
      rewards.append(self.evaluate(params))
    return np.mean(rewards)
  
  def evaluate(self, params: np.ndarray):
    obs = self.env.reset()
    done = False
    sum_reward = 0.0

    while not done:
      action = self.policy.call(params, obs)
      obs, reward, done, info = self.env.step(action)
      sum_reward += reward
    
    return sum_reward

class EnvARS(ARS):

  def __init__(
    self,
    env_creator: Callable[[],
    gym.Env],
    policy: Policy,
    config: ARSConfig,
    num_cpus: int = 1,
    num_eval_per_param: int=1
  ):

    self.num_cpus = num_cpus

    if not ray.is_initialized():
      ray.init(num_cpus=num_cpus)

    self.actors = [EnvActor.remote(env_creator, policy) for _ in range(num_cpus)]
    def evaluator(params: np.ndarray):
      results = []
      for i in range(0, len(params), num_cpus):
        sz = min(len(params) - i, num_cpus)
        refs = [self.actors[j].evaluate_n_times.remote(params[i+j], num_eval_per_param) for j in range(sz)]
        ray.wait(refs)
        results += [ray.get(x) for x in refs]
      return results

    super().__init__(evaluator, policy.init_params(), config)

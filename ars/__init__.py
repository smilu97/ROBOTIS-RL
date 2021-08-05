# author: smilu97

import gym
import numpy as np

from typing import Callable, List

from ars.config import ARSConfig
from ars.policy import Policy

class ARS:
  '''
  Reference: https://arxiv.org/pdf/1803.07055.pdf
  '''

  def __init__(self, evaluator: Callable[[np.ndarray], List[float]], init_params: np.ndarray, config: ARSConfig):
    self.evaluator = evaluator
    self.config = config

    self.param_size = init_params.shape[0]
    self.params = init_params
    self.j = 0
  
  def train(self):
    num_top_directions = self.config.num_top_directions
    num_directions     = self.config.num_directions
    step_size          = self.config.step_size

    # Sample random directions to explore
    diffs = self._select_random_diff(num_directions)

    # Evaluate the rewards to get from exploring
    params = np.concatenate([diffs, -diffs], axis=0) + self.params
    rewards = np.array(self.evaluator(params), dtype=np.float64).reshape((2, -1)).transpose()

    # Only b directions survive by their reward
    sort_keys    = [-max(r[0], r[1]) for r in rewards]
    sort_indices = np.argsort(sort_keys)[:num_top_directions]
    diffs = diffs[sort_indices]
    rewards = rewards[sort_indices]

    # Move parameters to the good enough directions
    std_reward = np.std(rewards)
    rewards = np.array([r[0] - r[1] for r in rewards], dtype=np.float64)
    updates = rewards.reshape((-1, 1)) * diffs
    mean_updates = np.mean(updates, axis=0)

    update = (step_size / std_reward) * mean_updates

    self.params += update
    self.j += 1
  
  def evaluate(self):
    return self.evaluator([self.params])[0]
  
  def _select_random_diff(self, n: int):
    return np.random.randn(n * self.param_size).reshape((n, self.param_size)) * self.config.exploration_noise

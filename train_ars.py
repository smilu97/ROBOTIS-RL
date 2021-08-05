#!/usr/bin/env python

#!/usr/bin/env python

import gym
import numpy as np

from ars.policy import Policy
from ars.config import ARSConfig
from ars.env_ars import EnvARS
from env import OP3Env

class LinearPolicy(Policy):
  def __init__(self, input_size: int, output_size: int, discrete_output=False):
    self.discrete_output = discrete_output
    self.input_size = input_size
    self.output_size = output_size
    self.shape = (input_size, output_size)

    self.param_size = input_size * output_size + output_size

    self.c_W = input_size * output_size

    super().__init__(self.param_size)

  def call(self, params, input):
    W = params[:self.c_W].reshape(self.shape)
    b = params[self.c_W:]
    r = np.matmul(input, W) + b
    if self.discrete_output:
      r = 1 if r[0] > 0.5 else 0
    r = np.minimum(1.0, np.maximum(-1.0, r))
    return r
  
  def init_params(self):
    # return np.array([-0.00191783, 0.08849352, 0.20041636, 0.27868464, 0.48853733], dtype=np.float64)
    return np.zeros(self.param_size, dtype=np.float64)

def main():
  def env_creator():
    return OP3Env(use_bias=True, human_bias=True)

  
  input_size = 24
  output_size = 12

  policy = LinearPolicy(input_size, output_size, discrete_output=False)
  config = ARSConfig(
    step_size=0.1,
    num_directions=100,
    num_top_directions=10,
    exploration_noise=0.01
  )
  
  ars = EnvARS(env_creator, policy, config, num_cpus=8)

  i = 0
  while True:
    score = np.mean([ars.evaluate(), ars.evaluate(), ars.evaluate(), ars.evaluate(), ars.evaluate()])
    print('score:', score)
    if score >= 500.0:
      break
    for _ in range(1):
      ars.train()
      i += 1
    
    if i % 10 == 0:
      print('save model')
      fd = open('result.npz', 'w')
      np.save(fd, ars.params)
      fd.close()
  
  print('params:', ars.params)

if __name__ == '__main__':
  main()

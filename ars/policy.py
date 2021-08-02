import numpy as np

class Policy:
  def __init__(self, param_size: int):
    '''
    Parameters
    ----------
    param_size: int
      the size of parameter vector
    '''

    self.param_size = param_size
  
  def call(self, params, input) -> np.ndarray:
    raise NotImplementedError('Policy: call method is not implemented')

  def init_params(self) -> np.ndarray:
    raise NotImplementedError('Policy: init_params method is not implemented')
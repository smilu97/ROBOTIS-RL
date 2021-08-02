class ARSConfig:
    def __init__(
      self,
      step_size: float,
      num_directions: int,
      num_top_directions: int,
      exploration_noise: float
    ):
      '''
        ARS hyper-parameters, and configurations

        Parameters
        ----------
        step_size: float
          the size of each steps for parameters to walk
        num_directions: int
          the number of directions to explore
        num_top_directions: int
          the number of directions to be selected
          only top `num_top_directions` of directions are applied by reward earning
        exploration_noise: float
          the difference between current parameters and the next parameters to explore
      '''

      self.step_size = step_size
      self.num_directions = num_directions
      self.num_top_directions = num_top_directions
      self.exploration_noise = exploration_noise


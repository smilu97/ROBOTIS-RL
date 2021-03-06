#!/usr/bin/env python

import ray
from ray import tune
from ray.rllib.agents.ppo import PPOTrainer
from ray.rllib.agents.sac import SACTrainer
from ray.tune.registry import register_env
from ppo_config import config as ppo_config
from sac_config import config as sac_config
from env import OP3Env
import gym
import pybulletgym

algo_trainer = 'sac'

config = {
    'ppo': ppo_config,
    'sac': sac_config,
}[algo_trainer]

def env_creator(env_config):
    return OP3Env(use_bias=True, human_bias=False)

register_env("RobotisOp3-v0", env_creator)
config["env"] = "RobotisOp3-v0"
config["num_workers"] = 1

class RayManager:
    def __init__(self):
        self.env_class = OP3Env
        self.config = config
        self.save_dir = '~/.ckp'
        self.env_config = {"use_bias": True, "human_bias": False}
        self.name = 'op3-21052611'

    def train(self, stop_criteria):
        """
        Train an RLlib PPO agent using tune until any of the configured stopping criteria is met.
        :param stop_criteria: Dict with stopping criteria.
            See https://docs.ray.io/en/latest/tune/api_docs/execution.html#tune-run
        :return: Return the path to the saved agent (checkpoint) and tune's ExperimentAnalysis object
            See https://docs.ray.io/en/latest/tune/api_docs/analysis.html#experimentanalysis-tune-experimentanalysis
        """
        trainer = {
            'ppo': PPOTrainer,
            'sac': SACTrainer,
        }[algo_trainer]
        analysis = ray.tune.run(
            trainer,
            config=self.config,
            local_dir=self.save_dir,
            stop=stop_criteria,
            checkpoint_at_end=True,
            name=self.name,
            resume=False,
        )
        # list of lists: one list per checkpoint; each checkpoint list contains 1st the path, 2nd the metric value
        checkpoints = analysis.get_trial_checkpoints_paths(trial=analysis.get_best_trial('episode_reward_mean'),
                                                        metric='episode_reward_mean')
        # retriev the checkpoint path; we only have a single checkpoint, so take the first one
        checkpoint_path = checkpoints[0][0]
        return checkpoint_path, analysis

    def load(self, path):
        """
        Load a trained RLlib agent from the specified path. Call this before testing a trained agent.
        :param path: Path pointing to the agent's saved checkpoint (only used for RLlib agents)
        """
        Trainer = {
            'ppo': PPOTrainer,
            'sac': SACTrainer,
        }[algo_trainer]
        self.agent = Trainer(config=self.config, env=self.env_class)
        self.agent.restore(path)

    def test(self, n=-1, render=True):
        """Test trained agent for a single episode. Return the episode reward"""
        # instantiate env class
        env = self.env_class(**self.env_config)

        if render:
            env.render(mode='human')

        while n != 0:
            # run until episode ends
            episode_reward = 0
            done = False
            obs = env.reset()
            while not done:
                action = self.agent.compute_action(obs)
                obs, reward, done, info = env.step(action)
                episode_reward += reward

            print('reward:', episode_reward)

            n -= (n > 0)

def main():
    path = '/home/smilu97/tmp/checkpoint-61260'
    ray.init()
    manager = RayManager()
    manager.load(path)
    manager.test()

if __name__ == '__main__':
    main()
config = {
    "num_workers": 8,
    "num_envs_per_worker": 1,
    "num_gpus": 1,
    "n_step": 1,
    "rollout_fragment_length": 1,
    "prioritized_replay": True,
    "batch_mode": "complete_episodes",
    "observation_filter": "MeanStdFilter",
    "Q_model": {
        "fcnet_activation": "relu",
        "fcnet_hiddens": [256, 256]
    },
    "policy_model": {
        "fcnet_activation": "relu",
        "fcnet_hiddens": [256, 256]
    },
    "tau": 0.005,
    "target_entropy": "auto",
    "target_network_update_freq": 1,
    "train_batch_size": 256,
    "timesteps_per_iteration": 1000,
    "learning_starts": 10000,
    "optimization": {
        "actor_learning_rate": 0.0003,
        "critic_learning_rate": 0.0003,
        "entropy_learning_rate": 0.0003,
    },
    "clip_actions": False,
    "normalize_actions": True,
    "evaluation_interval": 1,
    "metrics_smoothing_episodes": 5
}
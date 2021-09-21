config = {
    "num_workers": 8,
    "num_envs_per_worker": 1,
    "num_gpus": 1,
    "gamma": 0.99,
    "lambda": 0.95,
    "clip_param": 0.3,
    "kl_coeff": 1.0,
    "num_sgd_iter": 30,
    "lr": 0.0003,
    "sgd_minibatch_size": 128,
    "train_batch_size": 4000,
    "model": {
        "free_log_std": True
    },
    "entropy_coeff": 0.0001,
    "explore": True,
    "vf_clip_param": 100.0,
    "batch_mode": "truncate_episodes",
    "observation_filter": "MeanStdFilter",
    "model": {
        "fcnet_hiddens": [180, 120],
        "fcnet_activation": "relu",
        "use_lstm": False
    },
}
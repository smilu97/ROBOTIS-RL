config = {
    "num_workers": 8,
    "num_envs_per_worker": 1,
    "num_gpus": 1,
    "gamma": 0.995,
    "lambda": 0.95,
    "clip_param": 0.2,
    "kl_coeff": 1.0,
    "num_sgd_iter": 30,
    "lr": 0.0003,
    "sgd_minibatch_size": 6144,
    "train_batch_size": 65536,
    "model": {
        "free_log_std": True
    },
    "vf_clip_param": 100.0,
    "batch_mode": "complete_episodes",
    "observation_filter": "MeanStdFilter",
    "model": {
        "fcnet_hiddens": [225, 125],
        "fcnet_activation": "relu",
        "use_lstm": False
    },
}
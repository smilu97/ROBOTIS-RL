config = {
    "num_workers": 8,
    "num_envs_per_worker": 1,
    "num_gpus": 1,
    "gamma": 0.995,
    "lambda": 0.95,
    "clip_param": 0.2,
    "kl_coeff": 1.0,
    "num_sgd_iter": 20,
    "lr": 0.0001,
    "sgd_minibatch_size": 10000,
    "train_batch_size": 10000,
    "model": {
        "free_log_std": True
    },
    "batch_mode": "complete_episodes",
    "observation_filter": "MeanStdFilter"
}
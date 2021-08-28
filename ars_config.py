config = {
    "action_noise_std": 0.0,
    "noise_stdev": 0.02,  # std deviation of parameter noise
    "num_rollouts": 128,  # number of perturbs to try
    "rollouts_used": 32,  # number of perturbs to keep in gradient estimate
    "num_workers": 2,
    "sgd_stepsize": 0.01,  # sgd step-size
    "observation_filter": "MeanStdFilter",
    "noise_size": 250000000,
    "eval_prob": 0.03,  # probability of evaluating the parameter rewards
    "report_length": 10,  # how many of the last rewards we average over
    "offset": 0,
    # ARS will use Trainer's evaluation WorkerSet (if evaluation_interval > 0).
    # Therefore, we must be careful not to use more than 1 env per eval worker
    # (would break ARSPolicy's compute_action method) and to not do obs-
    # filtering.
    "evaluation_config": {
        "num_envs_per_worker": 1,
        "observation_filter": "NoFilter"
    },
}
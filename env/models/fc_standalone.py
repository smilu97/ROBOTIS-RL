import tensorflow as tf

from ray.rllib.models.tf.tf_modelv2 import TFModelV2

class FCStandalone(TFModelV2):
    def __init__(self, obs_space, action_space, num_outputs, model_config, name):
        super().__init__(obs_space, action_space, num_outputs, model_config, name)
        input_layer = tf.keras.layers.Input(shape=obs_space.shape)
        hidden_layer1 = tf.keras.layers.Dense(120, activation="relu")
        hidden_layer2 = tf.keras.layers.Dense(80, activation="relu")
        output_layer = tf.keras.layers.Dense(num_outputs)

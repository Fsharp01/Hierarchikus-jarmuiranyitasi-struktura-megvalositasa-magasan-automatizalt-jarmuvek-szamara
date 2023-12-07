import numpy as np
from tensorflow import keras
from classes.MapUtils import Utils

'''
Class for traffic predictor ANN inference
'''


class ModelInference:
    def __init__(self, model_path, key_file):
        self.model = keras.models.load_model(model_path)
        self.key = np.loadtxt(key_file).reshape((646, 1))
        # Elapsed time in the simulation between ticks in seconds
        self.delta_t = 3

    def run_inference(self, input_data, time_interval):

        prediction_temp = self.model.predict(input_data)
        prediction = self.key

        while time_interval >= 1:
            prediction_temp = self.model.predict(prediction_temp)
            prediction = np.append(prediction, prediction_temp.T[:, 0:1], axis=1)
            time_interval -= self.delta_t

        return prediction

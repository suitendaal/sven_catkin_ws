#!/usr/bin/python3

import numpy as np

class Predictor(object):
    """docstring for Predictor."""

    def __init__(self, config):
        super(Predictor, self).__init__()
        self.config = config

    def predict(self, data, window_length, time_step):
        order = min(len(data) - 1, self.config.order)
        if order == 0:
            return data[-1].value

        x = []
        y = []
        for j in data:
            x.append(j.time)
            y.append(j.value)
        coefs = np.polyfit(x, y, order)
        value = 0
        time = data[-1].time + time_step
        for j in range(len(coefs)):
            coef = coefs[j]
            value = value + coef * (time ** (len(coefs) - j - 1))
        return value

class PredictorConfiguration(object):
    """docstring for PredictorConfiguration."""

    def __init__(self, order=3):
        super(PredictorConfiguration, self).__init__()
        self.order = 3

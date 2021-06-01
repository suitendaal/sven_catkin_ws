#!/usr/bin/python3

class Bounder(object):
    """docstring for Bounder."""

    def __init__(self, config):
        super(Bounder, self).__init__()
        self.config = config

    def calc_bound(self, data, window_length, time_step):
        return self.config.bound

class BounderConfiguration(object):
    """docstring for BounderConfiguration."""

    def __init__(self, bound=0.1):
        super(BounderConfiguration, self).__init__()
        self.bound = bound

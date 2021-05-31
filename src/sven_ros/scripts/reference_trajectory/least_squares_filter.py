#!/usr/bin/python3

from filter import *
import numpy as np
from datapoint import *

class LeastSquaresFilter(Filter):
    """docstring for Filter."""

    def filter(self, data, window_length=None):
        if window_length is None:
            window_length = self.config.window_length

        result = []
        for i in range(len(data)):
            start = max(0, i - window_length)
            end = i + 1
            order = min(end - start - 1, self.config.order)
            if order == 0:
                result.append(data[i])
                continue
            subset = data[start:end]
            x = []
            y = []
            for j in subset:
                x.append(j.time)
                y.append(j.value)
            coefs = np.polyfit(x, y, order)
            value = 0
            time = data[i].time
            for j in range(len(coefs)):
                coef = coefs[j]
                value = value + coef * (time ** (len(coefs) - j - 1))
            result.append(DataPoint(time, value))
        return result


class LeastSquaresFilterConfiguration(FilterConfiguration):
    """docstring for FilterConfiguration."""

    def __init__(self, window_length=10, order=3):
        super().__init__(window_length)
        self.order = order

def main():
    data = [DataPoint(0,1), DataPoint(1,2), DataPoint(2,3), DataPoint(3,4), DataPoint(4,5), DataPoint(5,1), DataPoint(6,2), DataPoint(7,3), DataPoint(8,4), DataPoint(9,5)]
    config = LeastSquaresFilterConfiguration(window_length = 5, order = 3)
    filter = LeastSquaresFilter(config)
    print(filter.filter(data))

if __name__ == '__main__':
    main()

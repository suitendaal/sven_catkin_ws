#!/usr/bin/python3

from datapoint import DataPoint

class DataSet(list):
    """docstring for DataSet."""

    def __init__(self, *args):
        list.__init__(self, *args)

    def align_time(self, starting_time=0):
        for datapoint in self:
            datapoint.time = datapoint.timestamp - self[0].timestamp + starting_time


def main():
    data = DataSet()
    data.append(DataPoint(4,1))
    data.append(DataPoint(5,2))
    data.append(DataPoint(6,3))
    print(data)
    print(data[2])
    data[2] = DataPoint(6,4)
    print(data[2])
    data.align_time()
    print(data)

if __name__ == '__main__':
    main()

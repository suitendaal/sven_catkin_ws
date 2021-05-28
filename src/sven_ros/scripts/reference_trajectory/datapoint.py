#!/usr/bin/python3

class DataPoint(object):
    """docstring for DataPoint."""

    def __init__(self, timestamp, value):
        super(DataPoint, self).__init__()
        self.timestamp = timestamp
        self.time = timestamp
        self.value = value

    def __neg__(self):
        result = DataPoint(self.timestamp, -self.value)
        result.time = self.time
        return result

    def __add__(self, x):
        result = DataPoint(self.timestamp, self.value)
        if isinstance (x, DataPoint):
            result.value = self.value + x.value
        else:
            result.value = self.value + x
        return result

    def __sub__(self, x):
        return __add__(-x)

    def __eq__(self, x):
        if isinstance (x, DataPoint):
            return self.value == x.value
        else:
            return self.value == x

    def __ne__(self, x):
        return not (self == x)

    def __lt__(self, x):
        if isinstance (x, DataPoint):
            return self.value < x.value
        else:
            return self.value < x

    def __le__(self, x):
        if self < x:
            return true
        return self == x

    def __gt__(self, x):
        return not self <= x

    def __ge__(self, x):
        return not self < x

    def __getitem__(self, index):
        result = DataPoint(self.timestamp, self.value[index])
        result.time = self.time
        return result

    def __setitem__(self, index, value):
        self.value[index] = value

    def __str__(self):
        return "Time: {}, Value: {}".format(self.time, self.value)

    def __repr__(self):
        return str(self)

def main():
    a = DataPoint(0,4)
    b = DataPoint(1,5)
    c = a + b
    d = b + a
    e = DataPoint(3,"hoi")
    f = DataPoint(4,"hey")
    g = e + f
    print(a)
    print(b)
    print(c)
    print(d)
    print(e)
    print(f)
    print(g)

if __name__ == '__main__':
    main()

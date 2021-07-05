#ifndef DATAPOINT_H
#define DATAPOINT_H

struct DataPoint {
  double time;
  double value;
  DataPoint(double time, double value)
  : time(time), value(value) {}
};

#endif // DATAPOINT_H


#ifndef DATAPOINT_H
#define DATAPOINT_H

struct DataPoint {
  double time;
  std::vector<double> value;
  DataPoint(double time, std::vector<double> value)
  : time(time), value(value) {}
};

#endif // DATAPOINT_H


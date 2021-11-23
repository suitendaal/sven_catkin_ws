#ifndef PREDICTOR_H
#define PREDICTOR_H

#include "jump_detector_new/data_container.h"

class Predictor : public DataContainer {

 public:
  Predictor(unsigned int max_window_length)
  : DataContainer(max_window_length)
  {}
  
  Predictor()
  : DataContainer(0)
  {}
  
  virtual bool predict(const DataPoint &datapoint, std::vector<double> &value) const = 0;
};

#endif // LEAST_SQUARES_PREDICTOR_H


#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <vector>

#include "jump_detector/data_container.h"

class Predictor : public DataContainer {

  private:
  unsigned int order_;
  
  public:
  Predictor(unsigned int max_window_length)
  : DataContainer(max_window_length)
  {}
  
  Predictor()
  : DataContainer(0)
  {}
  
  virtual bool predict(DataPoint datapoint, double &value) const = 0;
};

#endif // LEAST_SQUARES_PREDICTOR_H


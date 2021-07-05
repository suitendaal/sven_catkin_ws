#ifndef PREDICTOR_H
#define PREDICTOR_H

#include <vector>

#include "jump_detector/datapoint.h"

class Predictor {

  private:
  
  public:
  virtual double predict(std::vector<DataPoint> data, double time) = 0;

};

#endif // PREDICTOR_H


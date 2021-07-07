#ifndef WEIGHTED_PREDICTOR_H
#define WEIGHTED_PREDICTOR_H

#include <vector>

#include "jump_detector/polynomial.h"
#include "jump_detector/predictor.h"

class WeightedPredictor : public Predictor {

  private:
  int order_;
  
  public:
  WeightedPredictor(int order)
  : order_(order)
  {}
  
  bool predict(std::vector<DataPoint> data, double time, double &value) {
    std::vector<double> times;
    std::vector<double> values;
    std::vector<double> tmp = polynomial::polyfit(times, values, order_);
  	if (data.size() >= order_) {
	  	value = order_;
	  	return true;
	  }
	  return false;
  }

};

#endif // WEIGHTED_PREDICTOR_H


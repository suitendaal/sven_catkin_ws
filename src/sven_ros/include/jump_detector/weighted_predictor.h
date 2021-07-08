#ifndef WEIGHTED_PREDICTOR_H
#define WEIGHTED_PREDICTOR_H

#include <vector>

#include "jump_detector/polynomial.h"
#include "jump_detector/predictor.h"

class WeightedPredictor : public Predictor {

  private:
  int order_;
  int n_not_analyzed_;
  int min_n_datapoints_;
  
  public:
  WeightedPredictor(int order)
  : order_(order),
  n_not_analyzed_(0),
  min_n_datapoints_(order)
  {}
  
  WeightedPredictor(int order, int n_not_analyzed)
  : order_(order),
  n_not_analyzed_(n_not_analyzed),
  min_n_datapoints_(order)
  {}
  
  WeightedPredictor(int order, int n_not_analyzed, int min_n_datapoints)
  : order_(order),
  n_not_analyzed_(n_not_analyzed),
  min_n_datapoints_(min_n_datapoints)
  {}
  
  bool predict(std::vector<DataPoint> data, double time, double &value) {
  
    // Remove last couple of datapoints
    std::vector<double> times;
    std::vector<double> values;
    for (int i = 0; i < (int)data.size() - n_not_analyzed_; i++) {
      
      times.push_back(data[i].time);
      values.push_back(data[i].value);
    }
    
    // Ensure enough datapoints
    if (times.size() < min_n_datapoints_) {
      return false;
    }
    
    // Initialize weights
    std::vector<double> weights(times.size());
    for (int i = 0; i < weights.size(); i++) {
      weights[i] = (double)(times.size() - i) / times.size();
    }
    
    // Evaluate polynomial
    std::vector<double> coefs = polynomial::polyfit(times, values, order_, weights);
  	value = polynomial::polyval(coefs, time);
  	return true;
  }

};

#endif // WEIGHTED_PREDICTOR_H


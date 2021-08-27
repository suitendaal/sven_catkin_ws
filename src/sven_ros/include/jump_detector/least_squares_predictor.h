#ifndef LEAST_SQUARES_PREDICTOR_H
#define LEAST_SQUARES_PREDICTOR_H

#include <vector>

#include "jump_detector/predictor.h"
#include "jump_detector/polynomial.h"

class LeastSquaresPredictor : public Predictor {

  private:
  unsigned int order_;
  
  public:
  LeastSquaresPredictor(unsigned int order, unsigned int max_window_length)
  : Predictor(max_window_length),
  order_(order)
  {}
  
  LeastSquaresPredictor(unsigned int order)
  : LeastSquaresPredictor(order, 0)
  {}
  
  virtual bool predict(const DataPoint &datapoint, double &value) const {
  	
  	// Check if prediction can be made
  	if (this->max_window_length_ < this->order_ + 1) {
  		return false;
  	}
  	
  	// Initialize time and value vector
  	std::vector<double> times;
    std::vector<double> values;
    std::vector<DataPoint> data = this->get_data();
    double time_shift = data[0].time;
    for (int i = 0; i < (int)data.size(); i++) {
      times.push_back(data[i].time - time_shift);
      values.push_back(data[i].value);
    }
    
    // Evaluate polynomial
    std::vector<double> coefs = polynomial::polyfit(times, values, this->order_);
  	value = polynomial::polyval(coefs, datapoint.time - time_shift);
  	return true;
  }
};

#endif // LEAST_SQUARES_PREDICTOR_H


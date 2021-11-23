#ifndef LEAST_SQUARES_PREDICTOR_H
#define LEAST_SQUARES_PREDICTOR_H

#include <vector>

#include "jump_detector_new/predictor.h"
#include "jump_detector_new/polynomial.h"

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
  
  virtual bool predict(const DataPoint &datapoint, std::vector<double> &value) const {
  	
  	// Check if prediction can be made
  	if (!this->enough_data(datapoint)) {
  		return false;
  	}
  	
  	// Initialize time and value vector
  	std::vector<double> times;
    std::vector<DataPoint> data = this->get_data();
    double time_shift = data[0].time;
    for (int j = 0; j < (int)data.size(); j++) {
	    times.push_back(data[j].time - time_shift);
	  }
    
    for (int i = 0; i < (int)datapoint.value.size(); i++)
    {
		  std::vector<double> values;
		  for (int j = 0; j < (int)data.size(); j++) {
		    values.push_back(data[j].value[i]);
		  }
		  
		  // Evaluate polynomial
		  std::vector<double> coefs = polynomial::polyfit(times, values, this->order_);
	    value.push_back(polynomial::polyval(coefs, datapoint.time - time_shift));
	  }
  	return true;
  }
  
  virtual bool enough_data(const DataPoint &datapoint) const {
    return this->get_data().size() >= this->order_ + 1;
  }
};

#endif // LEAST_SQUARES_PREDICTOR_H


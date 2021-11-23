#ifndef IMPACT_AWARE_FORCE_FILTER_H
#define IMPACT_AWARE_FORCE_FILTER_H

#include <iostream>

#include "jump_detector_new/jump_aware_filter.h"
#include "jump_detector_new/vec_distance.h"

class ImpactAwareForceFilter : public JumpAwareFilter {
	
public:

	ImpactAwareForceFilter(int max_window_length, Predictor &predictor, Bounder &bounder)
	: JumpAwareFilter(max_window_length, predictor, bounder)
	{}
	
	virtual bool detect_jump(const DataPoint &datapoint) {
		bool jump_detected = JumpAwareFilter::detect_jump(datapoint);
		
/*		std::cout << "Detect jumps" << std::endl;*/
/*		if (this->latest_value.size() > 0) {*/
/*			std::cout << this->latest_value[0] << ", " << this->latest_value[1] << ", " << this->latest_value[2] << std::endl;*/
/*		}*/
/*		if (this->latest_prediction.size() > 0) {*/
/*		  std::cout << this->latest_prediction[0] << ", " << this->latest_prediction[1] << ", " << this->latest_prediction[2] << std::endl;*/
/*		}*/
/*		if (this->latest_value.size() > 0 && this->latest_prediction.size()) {*/
/*		  std::cout << vec_distance::calc_distance(this->latest_prediction, this->latest_value) << ", " << this->latest_bound << std::endl;*/
/*		}*/
		
		return (jump_detected && vec_distance::magnitude(this->latest_value) > vec_distance::magnitude(this->latest_prediction));
	}
};

#endif // IMPACT_AWARE_FORCE_FILTER_H


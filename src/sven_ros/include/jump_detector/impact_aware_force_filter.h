#ifndef IMPACT_AWARE_FORCE_FILTER_H
#define IMPACT_AWARE_FORCE_FILTER_H

#include "jump_detector/jump_aware_filter.h"

class ImpactAwareForceFilter : public JumpAwareFilter {
	
public:

	ImpactAwareForceFilter(int max_window_length, Predictor &predictor, Bounder &bounder)
	: JumpAwareFilter(max_window_length, predictor, bounder)
	{}
	
	virtual bool detect_jump(const DataPoint &datapoint) {
		bool jump_detected = JumpAwareFilter::detect_jump(datapoint);
		return (jump_detected && this->latest_value > this->latest_prediction);
	}
};

#endif // IMPACT_AWARE_FORCE_FILTER_H


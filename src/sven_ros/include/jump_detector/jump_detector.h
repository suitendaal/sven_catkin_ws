#ifndef JUMP_DETECTOR_FILTER_H
#define JUMP_DETECTOR_FILTER_H

#include <vector>

#include "jump_detector/data_container.h"

class JumpDetector : public DataContainer {

	private:
    
  public:
    JumpDetector(unsigned int max_window_length)
    : DataContainer(max_window_length)
    {}
    
    // Processed an incoming datapoint. Returns true if a jump is detected.
    virtual bool update(double time, double value) {
    	DataPoint datapoint(time, value);
    	return this->update(datapoint);
    }
    
    // Processed an incoming datapoint. Returns true if a jump is detected.
    virtual bool update(DataPoint &datapoint) = 0;
    	
    virtual bool detect_jump(const DataPoint &datapoint) = 0;
};

#endif // JUMP_DETECTOR_FILTER_H


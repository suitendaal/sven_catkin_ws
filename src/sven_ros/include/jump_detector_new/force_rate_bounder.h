#ifndef FORCE_RATE_BOUNDER_H
#define FORCE_RATE_BOUNDER_H

#include "jump_detector_new/constant_bounder.h"

class ForceRateBounder : public ConstantBounder {
  
  public:
    ForceRateBounder(double bound_value, unsigned int max_window_length)
    : ConstantBounder(bound_value, max_window_length)
    {}
    
    ForceRateBounder(double bound_value)
    : ConstantBounder(bound_value)
    {}
    
    virtual bool bound(const DataPoint &datapoint, double &value) const {
      if (this->get_data().size() == 0) {
        return false;
      }
      value = (datapoint.time - this->get_data()[0].time) / this->get_data().size() * this->bound_;
      return true;
    }
};

#endif // FORCE_RATE_BOUNDER_H

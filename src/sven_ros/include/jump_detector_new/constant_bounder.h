#ifndef CONSTANT_BOUNDER_H
#define CONSTANT_BOUNDER_H

#include "jump_detector_new/bounder.h"

class ConstantBounder : public Bounder {

  protected:
    double bound_;
  
  public:
    ConstantBounder(double bound_value, unsigned int max_window_length)
    : Bounder(max_window_length),
    bound_(bound_value)
    {}
    
    ConstantBounder(double bound_value)
    : Bounder(),
    bound_(bound_value)
    {}
    
    virtual bool bound(const DataPoint &datapoint, double &value) const {
      value = bound_;
      return true;
    }
};

#endif // CONSTANT_BOUNDER_H

#ifndef CONSTANT_BOUNDER_H
#define CONSTANT_BOUNDER_H

#include "bounder.h"

class ConstantBounder : public Bounder {
  
  private:
    double bound_;
    
  public:
    ConstantBounder(double bound_value)
    : bound_(bound_value)
    {}
    
    bool bound(std::vector<DataPoint> data, double time, double &value) {
      if (data.size() > 0) {
        value = bound_;
        return true;
      }
      return false;
    }
};

#endif // CONSTANT_BOUNDER_H

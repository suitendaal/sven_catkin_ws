#ifndef BOUNDER_H
#define BOUNDER_H

#include <vector>

#include "jump_detector/datapoint.h"

class Bounder {

  private:
  
  public:
  virtual bool bound(std::vector<DataPoint> data, double time, double &value) = 0;

};

#endif // BOUNDER_H

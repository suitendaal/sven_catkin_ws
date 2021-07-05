#ifndef BOUNDER_H
#define BOUNDER_H

#include <vector>

#include "jump_detector/datapoint.h"

class Bounder {

  private:
  
  public:
  virtual double bound(std::vector<DataPoint> data, double time) = 0;

};

#endif // BOUNDER_H

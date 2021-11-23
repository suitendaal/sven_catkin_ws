#ifndef BOUNDER_H
#define BOUNDER_H

#include "jump_detector_new/data_container.h"

class Bounder : public DataContainer {
  
  public:
  	Bounder()
  	: DataContainer(0)
  	{}
  
    Bounder(unsigned int max_window_length)
    : DataContainer(max_window_length)
    {}
    
    virtual bool bound(const DataPoint &datapoint, double &value) const = 0;
};

#endif // BOUNDER_H

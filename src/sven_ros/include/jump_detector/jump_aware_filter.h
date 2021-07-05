#ifndef JUMP_AWARE_FILTER_H
#define JUMP_AWARE_FILTER_H

#include <vector>

#include "jump_detector/jump_detector.h"
#include "jump_detector/bounder.h"
#include "jump_detector/predictor.h"
#include "jump_detector/datapoint.h"

class JumpAwareFilter : public JumpDetector {

private:
    Predictor* predictor_;
    Bounder* bounder_;
    int window_length_;
    
  public:
    JumpAwareFilter(int max_window_length, Predictor* predictor, Bounder* bounder)
    : JumpDetector(max_window_length),
    window_length_(0),
    predictor_(predictor),
    bounder_(bounder)
    {}
    
    bool datapoint_arrived(double value, double time) {
      
      std::queue<DataPoint> q = data_;
      std::vector<DataPoint> v;
      while (!q.empty())
      {
        if (q.size() <= window_length_ + 1) {
          v.push_back(q.front());
        }
        q.pop();
      }
      double predicted_value = predictor_->predict(v, time);
      double bounded_value = bounder_->bound(v, time);
      
      bool result = abs(predicted_value - value) < bounded_value;
      
      if (result) {
        window_length_ = 0;
      }
      else if (window_length_ < max_window_length_) {
        window_length_++;
      }
      
      JumpDetector::datapoint_arrived(value, time);
      return result;
    }

};

#endif // JUMP_AWARE_FILTER_H


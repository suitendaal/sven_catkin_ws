#ifndef VEC_DISTANCE_H
#define VEC_DISTANCE_H

#include <cmath>
#include <vector>

namespace vec_distance {
  double calc_distance(std::vector<double> v1, std::vector<double> v2) {
    double value = 0;
    for (int i = 0; i < (int)v1.size(); i++)
    {
      value += pow(v1[i] - v2[i], 2);
    }
    return sqrt(value);
  }
  
  double magnitude(std::vector<double> v) {
  	double value = 0;
    for (int i = 0; i < (int)v.size(); i++)
    {
      value += pow(v[i], 2);
    }
    return sqrt(value);
  }
}

#endif // VEC_DISTANCE_H

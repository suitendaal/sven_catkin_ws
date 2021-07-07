#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <vector>
#include <Eigen/Dense>
#include <iostream>

namespace polynomial {
  
  // Stores a vector containing the coefficients of polyfit
  std::vector<double> polyfit(std::vector<double> times, std::vector<double> values, int order) {
    
    // Result
    std::vector<double> coefficients;
    
    // Vandermonde matrix
    Eigen::MatrixXd T(times.size(), order + 1);
    for (int i = 0; i < times.size(); i++) {
      T(i,0) = 1.0;
      for (int j = 1; j < order + 1; j++) {
        T(i,j) = T(i,j-1) * times[i];
      }
    }
    
    std::cout << T << std::endl;
    
    // Outcome vector
    Eigen::VectorXd y(values.size());
    for (int i = 0; i < values.size(); i++) {
      y(i) = values[i];
    }
    
    // Coefficients vector, doesnt go very well
    Eigen::VectorXd coefs = T.completeOrthogonalDecomposition().pseudoInverse().transpose() * y;

    for (int i = 0; i < order + 1; i++) {
      coefficients.push_back(coefs(i));
    }
    
    return coefficients;
  }
  
  // Returns a vector containing the coefficients of weighted polyfit
  std::vector<double> polyfit(std::vector<double> times, std::vector<double> values, int order, std::vector<double> weights) {
    for (int i = 0; i < times.size(); i++) {
      times[i] = times[i] * weights[i];
      values[i] = values[i] * weights[i];
    }
    return polyfit(times, values, order);
  }
  
  // Returns the evaluation of a polynomial
  double polyval(std::vector<double> coefficients, double time) {
    double result = coefficients[0];
    double time_coefficient = 1;
    for (int i = 1; i < coefficients.size(); i++) {
      time_coefficient *= time;
      result += coefficients[i] * time_coefficient;
    }
    return result;
  }
  
}

#endif // POLYNOMIAL_H

#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

#include <vector>
#include <Eigen/Dense>

namespace polynomial {

  std::vector<double> polyfit(std::vector<double> times, std::vector<double> values, int order);
  std::vector<double> polyfit(std::vector<double> times, std::vector<double> values, int order, std::vector<double> weights);
  double polyval(std::vector<double> coefficients, double time);

  
  // Stores a vector containing the coefficients of polyfit
  std::vector<double> polyfit(std::vector<double> times, std::vector<double> values, int order) {
    
    std::vector<double> weights(times.size());
    for (int i = 0; i < weights.size(); i++) {
      weights[i] = 1;
    }
    
    return polyfit(times, values, order, weights);
    
  }
  
  // Returns a vector containing the coefficients of weighted polyfit
  std::vector<double> polyfit(std::vector<double> times, std::vector<double> values, int order, std::vector<double> weights) {
    
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
    
    // Outcome vector
    Eigen::VectorXd y(values.size());
    for (int i = 0; i < values.size(); i++) {
      y(i) = values[i];
    }
    
    // Multiply with weights
    Eigen::VectorXd w(weights.size());
    for (int i = 0; i < weights.size(); i++) {
      w(i) = weights[i];
    }
    T.array().colwise() *= w.array();
    y.array() *= w.array();
    
    // Coefficients vector
    Eigen::VectorXd coefs(order + 1);
    Eigen::MatrixXd pinv(T.completeOrthogonalDecomposition().pseudoInverse());
    coefs << pinv * y;

    for (int i = 0; i < order + 1; i++) {
      coefficients.push_back(coefs(i));
    }
    
    return coefficients;
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

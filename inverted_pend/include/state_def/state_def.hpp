#ifndef STATE_DEF_HPP
#define STATE_DEF_HPP

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using Eigen::Matrix;
using std::cout;
using std::endl;


// State definition
// x = [theta, theta_dot, x, x_dot]
typedef Matrix<double, 4, 1> State;
typedef Matrix<double, 1, 4> Gain;
typedef Matrix<double, 4, 4> Mat44;

typedef Matrix<double, 2, 1> Mat21;
typedef Matrix<double, 2, 2> Mat22;



#endif // STATE_DEF_HPP
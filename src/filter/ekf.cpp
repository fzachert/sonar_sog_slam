/* ----------------------------------------------------------------------------
 * ekf.cpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides aa extended 3d kalmanfilter
 * ----------------------------------------------------------------------------
*/

#include "ekf.hpp"
#include <iostream>

using namespace std;
using namespace sonar_sog_slam;


void EKF::init(const base::Vector3d &mean, const base::Matrix3d &cov, double weight){
  
  this->state = mean;
  this->cov = cov;
  this->weight = weight;
  
}

#include "ekf.hpp"
#include <iostream>

using namespace std;
using namespace sonar_sog_slam;


void EKF::init(base::Vector3d mean, base::Matrix3d cov, double weight){
  
  this->state = mean;
  this->cov = cov;
  this->weight = weight;
  
}

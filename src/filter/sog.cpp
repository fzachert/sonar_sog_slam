#include "sog.hpp"
#include <iostream>

using namespace std;
using namespace sonar_sog_slam;

void SOG::welcome()
{
    cout << "You successfully compiled and executed DummyProject. Welcome!" << endl;
}


void SOG::init(base::Vector3d sensor_pos, base::Vector2d z, base::Matrix2d cov_z, double min_pitch_angle, double max_pitch_angle, int number_of_gaussians, double K){
  
  if(number_of_gaussians < 1)
    return;
  
  
  double angular_range = max_pitch_angle - min_pitch_angle;
  double angle_step = angular_range / ( (double) number_of_gaussians);
  double range = z.x();
  double angle = z.y();
  
  for( int i = 0; i < number_of_gaussians; i++){
    
    base::Vector3d mean = base::Vector3d::Zero();
    mean(0) = cos( angle) * cos( min_pitch_angle + ((i + 0.5) * angle_step )  );
    mean(1) = sin( angle) * cos( min_pitch_angle + ((i + 0.5) * angle_step )  ); 
    mean(2) = sin( min_pitch_angle + ((i + 0.5) * angle_step)  );
    
    base::Matrix3d cov = base::Matrix3d::Identity();
    cov(0,0) = cov_z(0,0);
    cov(1,1) =  std::pow( (std::sin( std::sqrt( cov_z(1,1)) ) * range), 2.0);
    cov(2,2) =  std::pow( std::sin(angle_step) * K * range , 2.0);
    
    //Calculate covariance rotation
    base::Vector3d a = base::Vector3d::UnitX();
    base::Vector3d b = mean.normalized();
    base::Vector3d v = a.cross(b);
    double s = v.norm();
    double c = a.dot(b);
    
    base::Matrix3d screw_v = base::Matrix3d::Zero();
    screw_v(0,1) = - v(2);
    screw_v(0,2) = v(1);
    screw_v(1,0) = v(2);
    screw_v(1,2) = -v(0);
    screw_v(2,0) = -v(1);
    screw_v(2,1) = v(0);
    
    base::Matrix3d rot = base::Matrix3d::Identity() + screw_v
      + ( (screw_v * screw_v) * ((1-c)/(s*s))  ); 
    
    EKF ekf;
    ekf.init( mean + sensor_pos, rot * cov * rot.transpose(), 1.0 / (double)number_of_gaussians );
   
    gaussians.push_back(ekf);
    
  } 
  
  
}

void SOG::reduceGaussians( double weight_threshold, double distance_threshold){
  
  if(gaussians.size() < 2)
    return;
  
  std::list<EKF> new_gaussians;
  std::list<EKF> temp;
  
  for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
    
    if(it->weight < weight_threshold){
      it = gaussians.erase(it);
      
      if(it!= gaussians.begin())
	it--;
      
    }    
  }  
  

  
  while(gaussians.size() > 0){
  
    EKF max_EKF;
    double max_weight = 0.0;
    
    for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
      
      if(it->weight > max_weight){
	
	max_EKF = *it;
	max_weight = it->weight;
      }    
      
    }
    
    temp.clear(); 
    double sum_weights = 0;
    base::Vector3d sum_means = base::Vector3d::Zero();
    base::Matrix3d sum_cov = base::Matrix3d::Zero();    
    
    for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
      
      double dist = std::sqrt((max_EKF.state - it->state).transpose() * max_EKF.cov.inverse() * (max_EKF.state - it->state) );
      
      if(dist < distance_threshold){
	temp.push_back(*it);
	
	sum_weights += it->weight;	
	sum_means += (it->weight * it->state);	
	
	it = gaussians.erase(it);
	
	if(it != gaussians.begin())
	  it--;
	
      }
      
    }
    
    EKF new_EKF;
    new_EKF.weight = sum_weights;
    new_EKF.state = sum_means / sum_weights;
    
    for(std::list<EKF>::iterator it = temp.begin(); it != temp.end(); it++){
      
      sum_cov += it->weight * ( it->cov + ( (new_EKF.state - it->state )*((new_EKF.state - it->state ).transpose())   ) );
      
    }  
    
    new_EKF.cov = sum_cov / sum_weights;
    new_gaussians.push_back(new_EKF);
  }
  
  gaussians = new_gaussians;
  
}



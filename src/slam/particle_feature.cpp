#include "particle.hpp"
#include <base/logging.h>


using namespace sonar_sog_slam;


void ParticleFeature::init(Eigen::Vector3d z, Eigen::Matrix3d cov_z, int number_of_gaussians, double K){
  
  if(number_of_gaussians < 1)
    return;
  
  double min_pitch_angle = base::getPitch(p->ori) 
      + p->model_config.sonar_vertical_angle + p->model_config.vertical_opening_angle;
  double angular_range = p->model_config.sonar_vertical_angle * 2.0;
  double angle_step = angular_range / ( (double) number_of_gaussians);
  double range = z.x();
  double angle = z.y() + base::getYaw(p->ori);
  
  for( int i = 0; i < number_of_gaussians; i++){
    
    base::Vector3d mean = base::Vector3d::Zero();
    mean(0) = range * cos( angle) * cos( min_pitch_angle + ((i + 0.5) * angle_step )  );
    mean(1) = range * sin( angle) * cos( min_pitch_angle + ((i + 0.5) * angle_step )  ); 
    mean(2) = range * sin( min_pitch_angle + ((i + 0.5) * angle_step)  );
    
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
    ekf.init( mean + p->pos + ( p->ori * p->model_config.sonar_pos) , rot * cov * rot.transpose(), 1.0 / (double)number_of_gaussians );
    ekf.counter = p->model_config.candidate_threshold;
    
    
    gaussians.push_back(ekf);
    
  } 
  
  
}



Eigen::Vector2d ParticleFeature::measurement_model_visable( base::Vector3d landmark){
  Eigen::Vector2d result;
  
  result(0) = (landmark - p->pos).norm();
  result(1) = std::atan2( landmark.y() - p->pos.y(), landmark.x() - p->pos.x() ) - (base::getYaw( p->global_orientation) + p->yaw_offset);
  
 return result; 
}

Eigen::Vector3d ParticleFeature::measurement_model_invisable( base::Vector3d landmark){
  Eigen::Vector3d result;
  
  result.x() = (landmark - p->pos).norm();
  result.y() = std::atan2( landmark.y() - p->pos.y(), landmark.x() - p->pos.x() ) - (base::getYaw( p->global_orientation) + p->yaw_offset);
  result.z() = std::atan2( landmark.z() - p->pos.z(), (landmark - p->pos).block<2,1>(0,0).norm() ) - ( base::getPitch( p->global_orientation) + p->model_config.sonar_vertical_angle);
  
  return result;
}

Eigen::Matrix<double, 2, 3> ParticleFeature::jacobi_measurement_model_visable( base::Vector3d landmark){

  Eigen::Matrix<double, 2, 3> result;
  double norm_l2z = (landmark - p->pos).norm();
  double squared_xy_norm = std::pow( landmark.x() - p->pos.x(), 2.0) + std::pow( landmark.y() - p->pos.y(), 2.0);
  
  result(0,0) = ( landmark.x() - p->pos.x() ) / norm_l2z;
  result(0,1) = ( landmark.y() - p->pos.y() ) / norm_l2z;
  result(0,2) = ( landmark.z() - p->pos.z() ) / norm_l2z;
  
  result(1,0) = ( -( landmark.y() - p->pos.y()) ) / squared_xy_norm;
  result(1,1) = ( landmark.x() - p->pos.x() ) / squared_xy_norm;  
  result(1,2) = 0.0;

  
  return result;
}
	    
Eigen::Matrix<double, 3, 3> ParticleFeature::jacobi_measurement_model_invisable( base::Vector3d landmark){
  
  Eigen::Matrix<double, 3, 3> result;
  double norm_l2z = (landmark - p->pos).norm();
  double squared_xy_norm = std::pow( landmark.x() - p->pos.x(), 2.0) + std::pow( landmark.y() - p->pos.y(), 2.0);
  double xy_norm = sqrt( squared_xy_norm);
  
  result(0,0) = ( landmark.x() - p->pos.x() ) / norm_l2z;
  result(0,1) = ( landmark.y() - p->pos.y() ) / norm_l2z;
  result(0,2) = ( landmark.z() - p->pos.z() ) / norm_l2z;
  
  result(1,0) = ( -( landmark.y() - p->pos.y()) ) / squared_xy_norm;
  result(1,1) = ( landmark.x() - p->pos.x() ) / squared_xy_norm;  
  result(1,2) = 0.0;  
  
  result(2,0) = (  (- ( landmark.z() - p->pos.z())) /  ( squared_xy_norm + std::pow( landmark.z() - p->pos.z(), 2.0) ) )
  *  ( (landmark.x() - p->pos.x()) / xy_norm );
  result(2,1) = (  (- ( landmark.z() - p->pos.z())) /  ( squared_xy_norm + std::pow( landmark.z() - p->pos.z(), 2.0) ) )
  *  ( (landmark.y() - p->pos.y()) / xy_norm );
  result(2,2) = xy_norm / ( squared_xy_norm + std::pow( landmark.z() - p->pos.z(), 2.0)  );
  
  return result;
}

bool ParticleFeature::isVisable( base::Vector3d landmark){
  
  double angle_vertical = std::atan2( landmark.z() - p->pos.z(), (landmark - p->pos).block<2,1>(0,0).norm() ) - base::getPitch( p->ori);
  
  if( angle_vertical < p->model_config.sonar_vertical_angle + p->model_config.vertical_opening_angle && angle_vertical > p->model_config.sonar_vertical_angle - p->model_config.vertical_opening_angle)
    return true;
  
  return false;
  
}

bool ParticleFeature::is_in_sensor_range(){
  
  base::Vector3d sensor_pos = p->pos + p->model_config.sonar_pos;
  bool visable = false;
  
  
  for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
    
    double range = (sensor_pos - it->state).norm();
    double angle_vertical = std::atan2( it->state.y() - sensor_pos.y(), it->state.x() - sensor_pos.x()) - base::getYaw( p->ori );
    double angle_horizontal = std::asin( (it->state.z() - sensor_pos.z()) / range) - base::getPitch( p->ori );
    
    if( range < p->model_config.max_range 
      && angle_vertical < p->model_config.sonar_vertical_angle + p->model_config.vertical_opening_angle
      && angle_vertical > p->model_config.sonar_vertical_angle - p->model_config.vertical_opening_angle
      && std::fabs( angle_horizontal ) < p->model_config.horizontal_opening_angle ){
      
      visable = true;
      it->visable = true;
    }else{
      it->visable = false;
    }
    
  }  

  return visable;
}

void ParticleFeature::setUnseen(){
  
  seen = false;
}


double ParticleFeature::negative_update(){  
  
  double negative_weight_sum = 0.0;
  
  for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
    
    if(it->visable){
      negative_weight_sum += it->weight;
      it->counter--;
      it->weight = it->weight * p->model_config.negative_likelihood;
      
      if(it->counter == 0){
	
	it = gaussians.erase(it);
	
	if(it != gaussians.begin())
	  it--;
	
      }      
      
    }
    
  }
  
    
  normalizeWeights();  
  
  return ( negative_weight_sum * p->model_config.negative_likelihood  ) 
    + ( ( 1.0 - negative_weight_sum ) * 1.0 );
}



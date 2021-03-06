/* ----------------------------------------------------------------------------
 * particle_feature.cpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a feature as part of an filter-particle
 * ----------------------------------------------------------------------------
*/

#include "particle_feature.hpp"
#include "../types/model_config.hpp"
#include <base/logging.h>


using namespace sonar_sog_slam;

ModelConfig ParticleFeature::model_config = ModelConfig();

void ParticleFeature::init(const Eigen::Vector3d &z, const Eigen::Matrix3d &cov_z, int number_of_gaussians, int initial_counter, double K){
  
  LOG_DEBUG_S << "Init feature with z=" << z.transpose() << " and " << number_of_gaussians << " gaussians";
  LOG_DEBUG_S << "p.pos=" << p->pos.transpose() << " p.ori= " << p->ori.w() << " " << p->ori.x() << " " << p->ori.y() << " " << p->ori.z();
  
  
  if(number_of_gaussians < 1)
    return;
  
  seen = true;
  changed = true;
  reduced = true;
  
  double min_pitch_angle = base::getPitch(p->ori) 
      + model_config.sonar_vertical_angle + model_config.vertical_opening_angle;
  double angular_range = model_config.sonar_vertical_angle * 2.0;
  double angle_step = angular_range / ( (double) number_of_gaussians);
  double range = z.x();
  double angle = z.y() + base::getYaw(p->ori);
  gaussians.clear();
  
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
    ekf.init( mean + p->pos + ( p->ori * model_config.sonar_pos) , rot * cov * rot.transpose(), 1.0 / (double)number_of_gaussians );
    ekf.counter = (initial_counter);
    
    
    gaussians.push_back(ekf);
    
  } 
  
  update_average_state();
}



Eigen::Vector2d ParticleFeature::measurement_model_visable( const base::Vector3d &landmark){
  Eigen::Vector2d result;
  base::Vector3d pos = p->pos + (p->ori * model_config.sonar_pos);
  
  result(0) = (landmark - pos).norm();
  result(1) = norm_angle( std::atan2( landmark.y() - pos.y(), landmark.x() - pos.x() ) - base::getYaw( p->ori) );
  
 return result; 
}

Eigen::Vector3d ParticleFeature::measurement_model_invisable( const base::Vector3d &landmark){
  Eigen::Vector3d result;
  base::Vector3d pos = p->pos + (p->ori * model_config.sonar_pos);
  
  result.x() = (landmark - pos).norm();
  result.y() = norm_angle( std::atan2( landmark.y() - pos.y(), landmark.x() - pos.x() ) - base::getYaw( p->ori) );
  result.z() = norm_angle( std::atan2( landmark.z() - pos.z(), (landmark - pos).block<2,1>(0,0).norm() ) - ( base::getPitch( p->ori) + model_config.sonar_vertical_angle) );
  
  return result;
}

Eigen::Matrix<double, 2, 3> ParticleFeature::jacobi_measurement_model_visable( const base::Vector3d &landmark){

  Eigen::Matrix<double, 2, 3> result;
  base::Vector3d pos = p->pos + (p->ori * model_config.sonar_pos);
  
  double norm_l2z = (landmark - pos).norm();
  double squared_xy_norm = std::pow( landmark.x() - pos.x(), 2.0) + std::pow( landmark.y() - pos.y(), 2.0);
  
  result(0,0) = ( landmark.x() - pos.x() ) / norm_l2z;
  result(0,1) = ( landmark.y() - pos.y() ) / norm_l2z;
  result(0,2) = ( landmark.z() - pos.z() ) / norm_l2z;
  
  result(1,0) = ( -( landmark.y() - pos.y()) ) / squared_xy_norm;
  result(1,1) = ( landmark.x() - pos.x() ) / squared_xy_norm;  
  result(1,2) = 0.0;

  
  return result;
}
	    
Eigen::Matrix<double, 3, 3> ParticleFeature::jacobi_measurement_model_invisable( const base::Vector3d &landmark){
  
  Eigen::Matrix<double, 3, 3> result;
  base::Vector3d pos = p->pos + (p->ori * model_config.sonar_pos);
  
  double norm_l2z = (landmark - pos).norm();
  double squared_xy_norm = std::pow( landmark.x() - pos.x(), 2.0) + std::pow( landmark.y() - pos.y(), 2.0);
  double xy_norm = sqrt( squared_xy_norm);
  
  result(0,0) = ( landmark.x() - pos.x() ) / norm_l2z;
  result(0,1) = ( landmark.y() - pos.y() ) / norm_l2z;
  result(0,2) = ( landmark.z() - pos.z() ) / norm_l2z;
  
  result(1,0) = ( -( landmark.y() - pos.y()) ) / squared_xy_norm;
  result(1,1) = ( landmark.x() - pos.x() ) / squared_xy_norm;  
  result(1,2) = 0.0;  
  
  result(2,0) = (  (- ( landmark.z() - pos.z())) /  ( squared_xy_norm + std::pow( landmark.z() - pos.z(), 2.0) ) )
  *  ( (landmark.x() - pos.x()) / xy_norm );
  result(2,1) = (  (- ( landmark.z() - pos.z())) /  ( squared_xy_norm + std::pow( landmark.z() - pos.z(), 2.0) ) )
  *  ( (landmark.y() - pos.y()) / xy_norm );
  result(2,2) = xy_norm / ( squared_xy_norm + std::pow( landmark.z() - pos.z(), 2.0)  );
  
  return result;
}

bool ParticleFeature::is_visable( const base::Vector3d &landmark){
  
  base::Vector3d pos = p->pos + (p->ori * model_config.sonar_pos);
  double angle_vertical = norm_angle(std::atan2( landmark.z() - pos.z(), (landmark - pos).block<2,1>(0,0).norm() ) - base::getPitch( p->ori) );
  
  if( angle_vertical < model_config.sonar_vertical_angle + model_config.vertical_opening_angle && angle_vertical > model_config.sonar_vertical_angle - model_config.vertical_opening_angle)
    return true;
  
  return false;
  
}

bool ParticleFeature::is_in_sensor_range(){
  
  base::Vector3d sensor_pos = p->pos + model_config.sonar_pos;
  bool visable = false;
  
  
  for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
    
    double range = (sensor_pos - it->state).norm();
    double angle_vertical = norm_angle(std::atan2( it->state.y() - sensor_pos.y(), it->state.x() - sensor_pos.x()) - base::getYaw( p->ori ) );
    double angle_horizontal = norm_angle( std::asin( (it->state.z() - sensor_pos.z()) / range) - base::getPitch( p->ori ) );
    
    if( range < model_config.max_range 
      && angle_vertical < model_config.sonar_vertical_angle + model_config.vertical_opening_angle
      && angle_vertical > model_config.sonar_vertical_angle - model_config.vertical_opening_angle
      && std::fabs( angle_horizontal ) < model_config.horizontal_opening_angle ){
      
      visable = true;
      it->visable = true;
    }else{
      it->visable = false;
    }
    
  }  

  return visable;
}

bool ParticleFeature::heuristic( const base::Vector3d &meas, const double &tolerance){
  
  base::Vector3d pos;
  base::Vector3d sensor_pos = p->pos + model_config.sonar_pos;
  double plane_range = meas.x() * cos( base::getPitch(p->ori) + model_config.sonar_vertical_angle);
  double yaw = base::getYaw( p->ori) + meas.y();
  pos.x() = sensor_pos.x() + ( plane_range * std::cos( yaw) );  
  pos.y() = sensor_pos.y() + ( plane_range * std::sin( yaw) );
  pos.z() = average_state.z();
  
  if( (average_state - pos).norm() < tolerance)
    return true;
  else
    return false;
  
}


void ParticleFeature::set_unseen(){
  
  seen = false;
}


double ParticleFeature::negative_update(){  
  
  double negative_weight_sum = 0.0;
  
  for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); ){
    
    if(it->visable){
      negative_weight_sum += it->weight;
      it->counter--;
      it->weight = it->weight * model_config.negative_likelihood;
      
      if(it->counter == 0){
	
	it = gaussians.erase(it);
	continue;
	
      }      
      
    }
    
    it++;
  }
  
    
  normalize_weights();  
  
  return ( negative_weight_sum * model_config.negative_likelihood  ) 
    + ( ( 1.0 - negative_weight_sum ) * 1.0 );
}

double ParticleFeature::norm_angle( double angle){
  
  if(angle > M_PI)
    return angle - M_PI;
  if(angle < -M_PI)
    return angle + M_PI;
  
  return angle;
  
}

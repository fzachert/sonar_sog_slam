/* ----------------------------------------------------------------------------
 * jacobian_test.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file tests the jocobians of the measurment-model, by calculating the numerical jacobian and checking the analytical
 * ----------------------------------------------------------------------------
*/

#include "sonar_sog_slam/slam/particle_feature.hpp"
#include "sonar_sog_slam/slam/particle.hpp"
#include "sonar_sog_slam/types/model_config.hpp"
#include <base/Eigen.hpp>
#include <iostream>
#include <math.h>

using namespace sonar_sog_slam;

bool check_jacobian_visable(base::Vector3d state){
  ModelConfig mc;
  Particle p;
  double dist;
  ParticleFeature pf;
  p.pos = base::Vector3d(1.0, 1.0, -3.0);
  p.ori = base::Orientation::Identity();
  
  pf.p = &p;
  
  mc.sonar_pos = base::Vector3d( 0.5, 0.0, 0.0);
  mc.sonar_vertical_angle = 0.2;
  mc.vertical_opening_angle = 0.3;
  mc.max_range = 16;
  mc.horizontal_opening_angle = 1.5;
  Particle::model_config = mc;
  
  base::Vector3d state_temp = state;
  
  Eigen::Matrix<double, 2,3> jacobi_num, jacobi_calc;
  double delta = 0.00000001;
  
  jacobi_calc = pf.jacobi_measurement_model_visable( state);
  
  for(int i = 0; i < 3; i++){
    state_temp = state;
    state_temp(i) = state(i) + delta;  
    
    base::Vector2d meas1 = pf.measurement_model_visable(state);
    base::Vector2d meas2 = pf.measurement_model_visable(state_temp);
    
    jacobi_num.block<2,1>(0,i) = (meas2 - meas1) / delta;
    
  }
  
  std::cout << "Measurement jocobian (numeric): " << std::endl;
  std::cout << jacobi_num << std::endl;
  std::cout << "Measurement jocobian (analyticaly): " << std::endl;
  std::cout << jacobi_calc << std::endl;  
  
  return jacobi_num.isApprox(jacobi_calc, 0.1);
}


bool check_jacobian_invisable(base::Vector3d state){
  
  ModelConfig mc;
  Particle p;
  double dist;
  ParticleFeature pf;
  p.pos = base::Vector3d(1.0, 1.0, -3.0);
  p.ori = base::Orientation::Identity();
  
  pf.p = &p;
  
  mc.sonar_pos = base::Vector3d( 0.5, 0.0, 0.0);
  mc.sonar_vertical_angle = 0.2;
  mc.vertical_opening_angle = 0.3;
  mc.max_range = 16;
  mc.horizontal_opening_angle = 1.5;
  
  base::Vector3d state_temp = state;
  
  Eigen::Matrix<double, 3,3> jacobi_num, jacobi_calc;
  double delta = 0.00000001;
  
  jacobi_calc = pf.jacobi_measurement_model_invisable( state);
  
  for(int i = 0; i < 3; i++){
    state_temp = state;
    state_temp(i) = state(i) + delta;  
    
    base::Vector3d meas1 = pf.measurement_model_invisable(state);
    base::Vector3d meas2 = pf.measurement_model_invisable(state_temp);
    
    jacobi_num.block<3,1>(0,i) = (meas2 - meas1) / delta;
    
  }
  
  std::cout << "Measurement jocobian (invisable, numeric): " << std::endl;
  std::cout << jacobi_num << std::endl;
  std::cout << "Measurement jocobian (invisable, analyticaly): " << std::endl;
  std::cout << jacobi_calc << std::endl;
  
  
  return jacobi_num.isApprox(jacobi_calc, 0.1);
}


int main (int argc, char *argv[]) { 
  
  int errors = 0;
  
  for( double d = -3; d < 3; d += 0.1){
  
    base::Vector3d state( std::cos(d) * 14.0, std::sin(d) * 5.0, -4.0);  
    std::cout << "----------------------------------------" << std::endl;
    std::cout << "State: " << state.transpose() << std::endl;
    if (check_jacobian_visable( state) && check_jacobian_invisable( state)){
            
    }else{
      std::cout << "Error in jacobian!!!" << std::endl;
      errors++;
    }
    
  }
  
  std::cout << "Got " << errors << " errors." << std::endl;
  
  return 0;
} 
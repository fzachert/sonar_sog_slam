/* ----------------------------------------------------------------------------
 * model_config.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a configuration of the measurement-model
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_MODELCONFIG_HPP_
#define _SOGSSLAM_MODELCONFIG_HPP_

#include <base/Eigen.hpp>
#include <base/Pose.hpp>

namespace sonar_sog_slam
{
    struct ModelConfig{

      //Variance-values
      base::Matrix3d sigmaZ;
      base::Matrix2d speed_covariance;
      base::Matrix2d dvl_dropout_covariance;
      double orientation_drift_variance;
      double negative_likelihood;
      
      //Properties of the sonar
      base::Vector3d sonar_pos;
      double sonar_vertical_angle; // in radian, angle of the scan-center in vehicle-frame
      double vertical_opening_angle; //in radian, angle from scancnter to upper scan-edge
      double max_range; //Maximum range of the sonar
      double horizontal_opening_angle; //Horizontal opening angel of the soar
      
      double dvl_dropout_threshold;
      
      base::Vector3d velocity_position; //Position of the velocity-sensor
      base::Orientation velocity_rotation; //Orientation off the velocity-sensor
      double max_velocity;

      
      
    };
    
}

#endif
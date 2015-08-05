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

      base::Matrix3d sigmaZ;
      double negative_likelihood;
      
      base::Vector3d sonar_pos;
      double sonar_vertical_angle; // in radian, angle of the scan-center in vehicle-frame
      double vertical_opening_angle; //in radian, angle from scancnter to upper scan-edge
      double max_range;
      double horizontal_opening_angle;
      
      int candidate_threshold;
      double candidate_distance;
      
      double heuristic_tolerance;
      
      double reduction_weight_threshold;
      double reduction_distance_threshold;
      double merge_distance_threshold;
      
      
    };
    
}

#endif
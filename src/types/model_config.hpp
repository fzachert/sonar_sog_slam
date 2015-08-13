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
      
      //Properties of the sonar
      base::Vector3d sonar_pos;
      double sonar_vertical_angle; // in radian, angle of the scan-center in vehicle-frame
      double vertical_opening_angle; //in radian, angle from scancnter to upper scan-edge
      double max_range; //Maximum range of the sonar
      double horizontal_opening_angle; //Horizontal opening angel of the soar
      
      //Properties of the candidate-filter
      int candidate_threshold; //A candidate neds to be observed x time, before considered valid
      double candidate_distance; //Distance, at which a measurement belongs to a feature
      
      double heuristic_tolerance; //Distance, at which a feature could have a matching correspondece to the messurement
      
      //Reuction properties
      double reduction_weight_threshold; //SOG-Elements with weight below this threshold will be removed
      double reduction_distance_threshold; //SOG-Elements, which are have a distance to each other below this threshold will be merged
      double reduction_trigger_probability; //With this probabilty, the SOG-reduction will be triggert
      
      double merge_distance_threshold; 
      
      
    };
    
}

#endif
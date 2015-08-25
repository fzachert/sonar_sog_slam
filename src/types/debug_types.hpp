/* ----------------------------------------------------------------------------
 * debug_types.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a debug-output of the slam
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_DEBUGTYPES_HPP_
#define _SOGSSLAM_DEBUGTYPES_HPP_

#include <base/Eigen.hpp>
#include <base/Pose.hpp>

namespace sonar_sog_slam
{
    struct DebugOutput{

      double effective_sample_size;
      int resample_count;
      double avg_observations_before_resampling;
      double avg_number_of_features;
      
      int best_number_of_features;
      double best_avg_number_of_gaussians;
      int best_max_number_of_gaussians;
      
      double avg_number_of_candidates;
      int max_number_of_candidates;
      
      int observation_count;
      double max_observation_time;
      double avg_observation_time;
      
      double yaw_offset_max;
      double yaw_offset_avg;
      
      double initial_feature_likelihood;
    };
    
}

#endif
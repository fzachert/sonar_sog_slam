/* ----------------------------------------------------------------------------
 * filer_config.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a configuration of the particle-filter
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_FILTERCONFIG_HPP_
#define _SOGSSLAM_FILTERCONFIG_HPP_

#include <base/Eigen.hpp>
#include <base/Pose.hpp>

namespace sonar_sog_slam
{
    struct FilterConfig{    
      
      int number_of_particles;
      int number_of_gaussians;
      double K;
      
      double new_feature_distance;
      
      double sonar_weight;
      
      double effective_sample_size_threshold;
      int min_observations; //Minimum number of observation before resampling
      bool use_markov;
      bool estimate_middle;
      
      
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
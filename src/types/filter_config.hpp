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
      double feature_penalty;
      double feature_penalty_maximum; //Maximum number of features
      
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
      
      FilterConfig() 
      : number_of_particles(100),
	number_of_gaussians(5),
	K(0.4),
	new_feature_distance(1.0),
	feature_penalty(0.0),
	feature_penalty_maximum(1.0),
	sonar_weight(1.0),
	effective_sample_size_threshold(1.0),
	min_observations(1),
	use_markov(true),
	estimate_middle(true),
	candidate_threshold(3),
	candidate_distance(1.0),
	heuristic_tolerance(5.0),
	reduction_weight_threshold(0.01),
	reduction_distance_threshold(0.1),
	reduction_trigger_probability(1.0),
	merge_distance_threshold(0.5){}     
      
    };
    
}

#endif
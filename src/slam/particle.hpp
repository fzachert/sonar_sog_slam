/* ----------------------------------------------------------------------------
 * particle.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a particle of the sog-slam-particle-filter
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_PARTICLE_HPP_
#define _SOGSSLAM_PARTICLE_HPP_

#include <list>
#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <base/samples/RigidBodyState.hpp>
#include "particle_feature.hpp"
#include "../maps/sog_map.hpp"
#include "../types/model_config.hpp"
#include "../filter/candidate_filter.hpp"


namespace sonar_sog_slam
{
    class ParticleFeature;
    class CandidateFilter;
  
    /**
     * A particle in the sog-particle-filter
     * Contains a particle-pose and a list of features and a filter for possible feature-candidates
     */
    class Particle    
    {
	public: 

	    base::Time time;
	  
	    std::list<ParticleFeature> features;	    
	    CandidateFilter candidate_filter;
	    
	    base::Vector3d pos;
	    base::Vector3d velocity;
	    double yaw_offset;
	    base::Orientation ori;
	    	    
	    double main_confidence;	    
	    
	    /**
	     * Returns the sog-map-representation
	     */
	    SOG_Map get_map( base::Vector3d transformation = base::Vector3d::Zero());
	    
	    /**
	     * Set all features as unseen
	     */
	    void set_unseen();
	    
	    /**
	     * Reduce the features of the particle
	     * If a feature was not seen in a while it will be removed
	     * If features a close to each other, they will be merged
	     */
	    void reduce_features(double merge_distance_threshold);
	    
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
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
  
    class Particle    
    {
	public: 

	    std::list<ParticleFeature> features;	    
	    CandidateFilter candidate_filter;
	    
	    base::Vector3d pos;
	    base::Vector3d velocity;
	    double yaw_offset;
	    base::Orientation ori;
	    
	    static base::Time time;
	    static ModelConfig model_config;
	    static base::Orientation global_orientation;
	    static double global_depth;
	    	    
	    double main_confidence;	    
	    
	    SOG_Map getMap();
	    void setUnseen();
	    
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
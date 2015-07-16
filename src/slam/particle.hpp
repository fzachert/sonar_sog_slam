#ifndef _SOGSSLAM_PARTICLE_HPP_
#define _SOGSSLAM_PARTICLE_HPP_

#include <list>
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include "particle_feature.hpp"
#include "../maps/sog_map.hpp"
#include "../types/model_config.hpp"

namespace sonar_sog_slam
{
    class Particle    
    {
	public: 

	    std::list<ParticleFeature> features;
	  
	    base::Vector3d pos;
	    base::Vector3d velocity;
	    double yaw_offset;
	    base::Orientation ori;
	    
	    static ModelConfig model_config;
	    static base::Orientation global_orientation;
	    static double global_depth;
	    
	    double main_confidence;	    
	    
	    SOG_Map getMap();
	    
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
#ifndef _SOGSSLAM_SOGSLAM_HPP_
#define _SOGSSLAM_SOGSLAM_HPP_

#include <base/Eigen.hpp>
#include "../filter/sog.hpp"
#include "../types/slam_particle.hpp"
#include <uw_localization/filter/particle_filter>

namespace sonar_sog_slam
{
    class sog_slam : public uw_localization::ParticleFilter<SlamParticle>,
	public Perception<SlamParticle, base::Vector3d, void>,
	public Dynamic<SlamParticle, base::samples::RigidBodyState, void>  
    
    {
	private: 

	public:
	  
	  virtual base::Position position(const SlamParticle& X) const { return X.position; }
	  virtual base::Vector3d velocity(const SlamParticle& X) const { return X.velocity; }
	  virtual bool isValid(const SlamParticle& X) const {return X.valid; }
	  virtual void setValid(SlamParticle &X, bool flag){ X.valid = flag; }
	  
	  virtual double confidence( const SlamParticle& X) const {return X.confidence; }
	  virtual setConfidence(SlamParticle& X, double weight) {X.confidence = weight; }
	  
	  
	  virtual void dynamic(SlamParticle X, base::samples::RigidBodyState u, void);
	    
	  virtual double perception(SlamPparticle X, base::Vector3d z, void);  
	  
	  
	  
	  double observe( const base::Vector3d z);
	  
	    
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
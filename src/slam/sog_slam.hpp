#ifndef _SOGSSLAM_SOGSLAM_HPP_
#define _SOGSSLAM_SOGSLAM_HPP_

#include <base/Eigen.hpp>
#include "../filter/sog.hpp"
#include "../types/filter_config.hpp"
#include "../types/model_config.hpp"
#include "../maps/sog_map.hpp"
#include "particle.hpp"
#include <uw_localization/filters/particle_filter.hpp>
#include <sonar_image_feature_extractor/DetectorTypes.hpp>
#include <machine_learning/RandomNumbers.hpp>

namespace sonar_sog_slam
{
    class SOG_Slam : public uw_localization::ParticleFilter<Particle>,
	public uw_localization::Perception<Particle, sonar_image_feature_extractor::SonarFeatures, DummyMap>,
	public uw_localization::Dynamic<Particle, base::samples::RigidBodyState, DummyMap>  
    
    {
	private: 
	  
	  machine_learning::MultiNormalRandom<2> *StaticSpeedNoise;
	  machine_learning::MultiNormalRandom<1> *StaticOrientationNoise;
	  base::Time last_velocity_sample;
	  
	  
	  FilterConfig config;
	  ModelConfig model_config;
	  
	  void init_particles( unsigned int number_of_particles);
	  
	  double perception_positive(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z);
	  
	  double perception_negative(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z);
	  
	  
	public:
	  
	  SOG_Slam();
	  ~SOG_Slam();
	  
	  void init(FilterConfig config, ModelConfig mconfig);
	  
	  virtual base::Position position(const Particle& X) const { return X.pos; }
	  virtual base::Vector3d velocity(const Particle& X) const { return base::Vector3d::Zero(); }
	  virtual bool isValid(const Particle& X) const {return true; }
	  virtual void setValid(Particle &X, bool flag){ }
	  
	  virtual double confidence( const Particle& X) const {return X.main_confidence; }
	  virtual void setConfidence(Particle& X, double weight) {X.main_confidence = weight; }
	  
	  
	  virtual void dynamic(Particle &X, const base::samples::RigidBodyState &u, DummyMap &m);
	    
	  virtual double perception(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z, DummyMap &m);  
	    
	  
	  double observeFeatures( const sonar_image_feature_extractor::SonarFeatures &z, double weight);
	  
	    
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
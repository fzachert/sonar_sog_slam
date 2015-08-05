/* ----------------------------------------------------------------------------
 * sog_slam.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file implements the sog-slam, using a particle-filter
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_SOGSLAM_HPP_
#define _SOGSSLAM_SOGSLAM_HPP_

#include <base/Eigen.hpp>
#include "../filter/sog.hpp"
#include "../types/filter_config.hpp"
#include "../types/model_config.hpp"
#include "../types/debug_types.hpp"
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
	  
	  //random-number for the dynamic-model
	  boost::minstd_rand *seed;
	  machine_learning::MultiNormalRandom<2> *StaticSpeedNoise;
	  machine_learning::MultiNormalRandom<1> *StaticOrientationNoise;
	  
	  base::Time last_velocity_sample;
	  
	  
	  FilterConfig config;
	  ModelConfig model_config;
	  DebugOutput debug;
	  double initial_feature_likelihood;
	  
	  double global_depth;
	  base::Orientation global_orientation;
	  
	  /**
	   * Initialize the particles with a given number
	   */
	  void init_particles( unsigned int number_of_particles);
	  
	  double calculate_initial_feature_likelihood(); 
	  
	  
	  /**
	   * Positive perception
	   * Apply measurement to the feature based on the maximum likelihood
	   *  or create new features
	   * @param X: Particle
	   * @param z: Sonar-measurement
	   * @return: Likelihood of the measurement
	   */
	  double perception_positive(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z);
	  
	  /**
	   * Negative perception
	   * Correct the unseen features in the sensor-range and remove unlikely features
	   * @param X: Particle
	   * @param z: Sonar-measurement
	   * @return: Likelihood of not seing features
	   */
	  double perception_negative(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z);
	  
	  
	public:
	  
	  SOG_Slam();
	  ~SOG_Slam();
	  
	  /**
	   * Initialize the slam with a given config
	   */
	  void init(FilterConfig config, ModelConfig mconfig);
	  
	  /**
	   * Implemented virtual functions of the particle-filter-template
	   */
	  virtual base::Position position(const Particle& X) const { return X.pos; }
	  virtual base::Vector3d velocity(const Particle& X) const { return X.velocity; }
	  virtual base::samples::RigidBodyState orientation(const Particle& X) const {
	    base::samples::RigidBodyState rbs;
	    rbs.time = X.time;
	    rbs.orientation = X.ori;
	    rbs.position.z() = global_depth;
	    return rbs;
	  }
	  virtual const base::Time& getTimestamp(const base::samples::RigidBodyState& motion) { return motion.time; }
	  virtual bool isValid(const Particle& X) const {return true; }
	  virtual void setValid(Particle &X, bool flag){ }
	  
	  virtual double confidence( const Particle& X) const {return X.main_confidence; }
	  virtual void setConfidence(Particle& X, double weight) {X.main_confidence = weight; }	  
	  
	  virtual void dynamic(Particle &X, const base::samples::RigidBodyState &u, const DummyMap &m);
	    
	  virtual double perception(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z, DummyMap &m);  
	    
	  
	  /**
	   * Observe a feature
	   * Calls the measurement-function of the particle-filter and triggers the resampling
	   */
	  double observe_features( const sonar_image_feature_extractor::SonarFeatures &z, double weight);
	  
	  /**
	   * Set the epth of all particles
	   */
	  void set_depth( const double &depth);
	  
	  /**
	   * Set the orientation of all particles
	   */
	  void set_orientation(  const base::Orientation &ori);
	  
	  /**
	   * Get the map-representation of the best particle
	   */
	  SOG_Map get_map();
	  
	  /**
	   * Get debug-informations of the slam and particle-filter
	   */
	  DebugOutput get_debug();
	  
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
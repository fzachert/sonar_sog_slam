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
#include "../types/dvl_dropout.hpp"
#include "../maps/sog_map.hpp"
#include "particle.hpp"
#include <uw_localization/filters/particle_filter.hpp>
#include <sonar_image_feature_extractor/DetectorTypes.hpp>
#include <machine_learning/RandomNumbers.hpp>
#include <stdlib.h> 
#include <time.h>
#include <fenv.h> //Nan-exceptions

namespace sonar_sog_slam
{
    class SOG_Slam : public uw_localization::ParticleFilter<Particle>,
	public uw_localization::Perception<Particle, sonar_image_feature_extractor::SonarFeatures, DummyMap>,
	public uw_localization::Perception<Particle, base::samples::RigidBodyState, DummyMap>,  
	public uw_localization::Dynamic<Particle, base::samples::RigidBodyState, DummyMap>,  
	public uw_localization::Dynamic<Particle, DvlDropoutSample, DummyMap>
    {
	private: 
	  
	  //random-number for the dynamic-model
	  boost::minstd_rand *seed;
	  machine_learning::MultiNormalRandom<2> *StaticSpeedNoise;
	  machine_learning::MultiNormalRandom<2> *StaticDropoutNoise;
	  machine_learning::MultiNormalRandom<1> *StaticOrientationNoise;
	  
	  base::Time last_velocity_sample;
	  
	  
	  FilterConfig config;
	  ModelConfig model_config;
	  DebugOutput debug;
	  double initial_feature_likelihood;
	  
	  int observation_count;
	  
	  double global_depth;
	  base::Orientation global_orientation;
	  base::Time global_time;
	  
	  base::samples::RigidBodyState dead_reackoning_state; 
	  
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
	  
	  /**
	   * Random number between 0 and 1
	   */
	  double double_rand();
	  
	  
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
	    rbs.time = global_time;
	    rbs.orientation = ( Eigen::AngleAxisd( X.yaw_offset, base::Vector3d::UnitZ()) * global_orientation);
	    rbs.position.z() = global_depth;
	    return rbs;
	  }
	  virtual const base::Time& getTimestamp(const base::samples::RigidBodyState& motion) { return motion.time; }
	  virtual const base::Time& getTimestamp(const DvlDropoutSample& motion) { return motion.time; }
	  virtual bool isValid(const Particle& X) const {return true; }
	  virtual void setValid(Particle &X, bool flag){ }
	  
	  virtual double confidence( const Particle& X) const {return X.main_confidence; }
	  virtual void setConfidence(Particle& X, double weight) {X.main_confidence = weight; }	  
	  
	  virtual void dynamic(Particle &X, const base::samples::RigidBodyState &u, const DummyMap &m);
	  virtual void dynamic(Particle &X, const DvlDropoutSample &u, const DummyMap &m);
	  
	  virtual double perception(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z, DummyMap &m);  
	  virtual double perception(Particle &X, const base::samples::RigidBodyState &z, DummyMap &m);
	  
 	  void update_dead_reackoning( const base::samples::RigidBodyState &velocity); 
	  base::samples::RigidBodyState estimate_dead_reackoning();
	  
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
	   * Set the global time for all particle-samples
	   */
	  void set_time( const base::Time &time);
	  
	  /**
	   * Get the map-representation of the best particle
	   */
	  SOG_Map get_map( base::Vector3d transformation = base::Vector3d::Zero());
	  
	  /**
	   * Get debug-informations of the slam and particle-filter
	   */
	  DebugOutput get_debug();
	  
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
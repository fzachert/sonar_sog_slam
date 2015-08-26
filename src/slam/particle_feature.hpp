/* ----------------------------------------------------------------------------
 * particle_feature.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a feature as part of an filter-particle
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_PARTICLEFEATURE_HPP_
#define _SOGSSLAM_PARTICLEFEATURE_HPP_

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/Pose.hpp>
#include "../filter/sog.hpp"
#include "../types/slam_particle.hpp"
#include "../types/model_config.hpp"
#include "../maps/sog_map.hpp"
#include "particle.hpp"

namespace sonar_sog_slam
{
    class Particle;
  
    /**
     * A feature, represented as a sum of gaussian filter
     */
    class ParticleFeature : public SOG<2,3>    
    {
	public: 
	  
	  static ModelConfig model_config;
	  
	  Particle *p;	  
	  
	    bool seen;
	  
	    virtual void init(const Eigen::Vector3d &z, const Eigen::Matrix3d &cov_z, int number_of_gaussians, int initial_counter, double K = 0.4);
	  	    
	    virtual Eigen::Vector2d measurement_model_visable( const base::Vector3d &landmark);
	    
	    virtual Eigen::Vector3d measurement_model_invisable( const base::Vector3d &landmark);
	    
	    virtual Eigen::Matrix<double, 2, 3> jacobi_measurement_model_visable( const base::Vector3d &landmark);
	    
	    virtual Eigen::Matrix<double, 3, 3> jacobi_measurement_model_invisable( const base::Vector3d &landmark);		    
	  
	    virtual bool is_visable( const base::Vector3d &landmark) ; 
	    
	    /**
	     * Checks, if the feature is in sensor-is_in_sensor_range
	     */
	    bool is_in_sensor_range();
	    
	    /**
	     * Simple heuristic, to check if the feature is near to  measurement
	     * @param meas: Measurement vector
	     * @param tolerance: Tolerance of the measurement
	     * @return: true, if the measurement is near to the feature
	     */
	    bool heuristic( const base::Vector3d &meas, const double &tolerance);
	    
	    
	    /**
	     * Negative update of the feature
	     * Reduceses the weight of all gaussians, which should be seens, but were not seen
	     */
	    double negative_update();
	    
	    /**
	     * Set all gaussians as unseen
	     */
	    void set_unseen();
	    
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
#ifndef _SOGSSLAM_PARTICLEFEATURE_HPP_
#define _SOGSSLAM_PARTICLEFEATURE_HPP_

#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/Pose.hpp>
#include "../filter/sog.hpp"
#include "../types/slam_particle.hpp"
#include "../maps/sog_map.hpp"
#include "particle.hpp"

namespace sonar_sog_slam
{
    class Particle;
  
  
    class ParticleFeature : public SOG<2,3>    
    {
	private: 
	  Particle *p;
	  	  
	  
	public:
	  
	    bool seen;
	  
	  
	    virtual void init(Eigen::Vector3d z, Eigen::Matrix3d cov_z, int number_of_gaussians, double K = 0.4);
	  	    
	    virtual Eigen::Vector2d measurement_model_visable( base::Vector3d landmark);
	    
	    virtual Eigen::Vector3d measurement_model_invisable( base::Vector3d landmark);
	    
	    virtual Eigen::Matrix<double, 2, 3> jacobi_measurement_model_visable( base::Vector3d landmark);
	    
	    virtual Eigen::Matrix<double, 3, 3> jacobi_measurement_model_invisable( base::Vector3d landmark);		    
	  
	    virtual bool isVisable( base::Vector3d landmark) ; 
	    
	    bool is_in_sensor_range();
	    
	    void setUnsee();
	    
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
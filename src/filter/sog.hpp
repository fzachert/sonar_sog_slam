#ifndef _SOGSSLAM_SOG_HPP_
#define _SOGSSLAM_SOG_HPP_

#include <base/Eigen.hpp>
#include "ekf.hpp"
#include <list>
#include <math.h>

namespace sonar_sog_slam
{
    class SOG
    {
	private:
	  std::list<EKF> gaussians;
          
	  void updateWeights();
	  
	  void normalizeWeights();
	  
	  
        public: 
            /**
             * Print a welcome to stdout
             * \return nothing
             */
            void welcome();
	    

	    void init(base::Vector3d sensor_pos, base::Vector2d z, base::Matrix2d cov_z, double min_pitch_angle, double max_pitch_angle, int number_of_gaussians, double K = 0.4);
	    
	    template <unsigned int INPUT_SIZE>
	    double calcLikelihood( Eigen::Matrix<double, INPUT_SIZE, 1> z, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> H){
	      	      
	    }
	    
	    template <unsigned int INPUT_SIZE>
	    void update( Eigen::Matrix<double, INPUT_SIZE, 1> z, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> H){
	      
	      
	    }
	    
	    void negativeUpdate();
	    
	    template <unsigned int INPUT_SIZE>
	    void dynamic( Eigen::Matrix<double, INPUT_SIZE, 1> u, Eigen::Matrix3d H_X, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> H_U){
	      
	      
	    }
	    
	    bool isVisable( double minPitchAngle, double maxPitchAngle, double minYawAngle, double maxYawAngle);
	    
	    void reduceGaussians( double weight_threshold, double distance_threshold);
	    
	    
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
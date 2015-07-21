#ifndef _SOGSSLAM_FILTERCONFIG_HPP_
#define _SOGSSLAM_FILTERCONFIG_HPP_

#include <base/Eigen.hpp>
#include <base/Pose.hpp>

namespace sonar_sog_slam
{
    struct FilterConfig{

      base::Matrix2d speed_covariance;
      double orientation_drift_variance;
      base::Matrix3d sigmaZ;      
      
      int number_of_particles;
      int number_of_gaussians;
      double K;
      
      double new_feature_probability;
      
      double sonar_weight;
      
      double effective_sample_size_threshold;
      bool use_markov;
      
    };
    
}

#endif
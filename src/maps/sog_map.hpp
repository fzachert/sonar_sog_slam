#ifndef _SOGSSLAM_SOGMAP_HPP_
#define _SOGSSLAM_SOGMAP_HPP_

#include <base/Eigen.hpp>
#include <base/Time.hpp>
#include <vector>

namespace sonar_sog_slam
{
    struct Gaussian{
      
      base::Vector3d mean;
      base::Matrix3d cov;
      double weight;
      
    };
  
  
    struct SOG_Feature{
      std::vector<Gaussian> gaussians;      
    };  
  
    struct SOG_Map{
      base::Time time;           
      std::vector<SOG_Feature> features;
      
    };
    
    struct DummyMap{
      
    };
    
}

#endif
/* ----------------------------------------------------------------------------
 * sog_map.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a output-representation of the sog-filter-map
 * ----------------------------------------------------------------------------
*/

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
    
    struct Simple_Feature{
      base::Vector3d pos;
      int descriptor;
    };
  
    struct SOG_Map{
      base::Time time;           
      std::vector<SOG_Feature> features;
      std::vector<Simple_Feature> simple_features;
      
    };
    
    struct DummyMap{
      
    };
    
}

#endif
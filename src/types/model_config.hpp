#ifndef _SOGSSLAM_MODELCONFIG_HPP_
#define _SOGSSLAM_MODELCONFIG_HPP_

#include <base/Eigen.hpp>
#include <base/Pose.hpp>

namespace sonar_sog_slam
{
    struct ModelConfig{

      base::Matrix3d sigmaZ;
      base::Vector3d sonar_pos;
      double sonar_vertical_angle; // in radian, angle of the scan-center in vehicle-frame
      double vertical_opening_angle; //in radian, angle from scancnter to upper scan-edge
      double max_range;
      double horizontal_opening_angle;
      
    };
    
}

#endif
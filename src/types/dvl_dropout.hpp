/* ----------------------------------------------------------------------------
 * dvl_dropout.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a debug-output of the slam
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_DVLDROPOUTSAMPLE_HPP_
#define _SOGSSLAM_DVLDROPOUTSAMPLE_HPP_

#include <base/Time.hpp>

namespace sonar_sog_slam
{
    struct DvlDropoutSample{

      base::Time time;
    };
    
}

#endif
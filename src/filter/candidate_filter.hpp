/* ----------------------------------------------------------------------------
 * candidate_filter.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a candidate filter, which rejects outliers on a particle-basis
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_CANDIDATE_FILTER_HPP_
#define _SOGSSLAM_CANDIDATE_FILTER_HPP_
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <math.h>
#include <machine_learning/GaussianParameters.hpp>
#include <list>
#include "../types/model_config.hpp"
//#include "../slam/particle.hpp"

namespace sonar_sog_slam
{
    class Particle;  
  
    /**
     * A single candidate, of a possible mapfeature
     */
    struct Candidate{
      base::Vector3d pos;
      int confidence;
      bool seen;
    };
  
  
    /**
     * This filter checks, if a feature is a possible mapfeature or an outlier
     */
    class CandidateFilter
    {
          
    private:  
      ModelConfig config;      
      
    public:
      
      std::list<Candidate> candidates;
      
      /**
       * Initializationfunktion
       * @param config: Configuration of the measurement-model
       */
      void init(ModelConfig config);
      
      /**
       * Checks, if a measurement corresponds with an earlier observed feature
       * If the a feature was observed multiple, it is valid
       * @param range: range-measurement of the feature [m]
       * @param alpha_h: horizotal angle of the feature [rad]
       * @param p: pointer to the particle, which observed the feature
       * @return: true, if the feature was observed multiple time and seems to be valid  
       */
      bool check_candidate( double range, double alpha_h, Particle *p);
      
      /**
       * Reduce candidates, if they were not observed
       */
      void reduce_candidates();
    };
    
}
#endif
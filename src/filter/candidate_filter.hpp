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
  
    struct Candidate{
      base::Vector3d pos;
      int confidence;
      bool seen;
    };
  
  
    class CandidateFilter
    {
          
    private:  
      ModelConfig config;
      std::list<Candidate> candidates;
      
    public:
      void init(ModelConfig config);
      
      bool check_candidate( double range, double alpha_h, Particle *p);
      
      void reduce_candidates();
    };
    
}
#endif
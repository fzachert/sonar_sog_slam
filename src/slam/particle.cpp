#include "particle.hpp"
#include <base/logging.h>

using namespace sonar_sog_slam;

ModelConfig Particle::model_config = ModelConfig();
base::Orientation Particle::global_orientation = base::Orientation::Identity();
double Particle::global_depth = 0.0;

SOG_Map Particle::getMap(){
  SOG_Map map;
  map.time = this->time;
  
  for(std::list<ParticleFeature>::iterator it = features.begin(); it != features.end(); it++){
    SOG_Feature sf;    
    
    for( std::list<EKF>::iterator it_ekf = it->gaussians.begin(); it_ekf != it->gaussians.end(); it_ekf++){
      Gaussian g;
      g.mean = it_ekf->state;
      g.cov = it_ekf->cov;
      g.weight = it_ekf->weight;
      sf.gaussians.push_back(g);
    }
    
    map.features.push_back(sf);
  } 
  
  return map;
}

void Particle::setUnseen(){
  
  for(std::list<ParticleFeature>::iterator it = features.begin(); it != features.end(); it++){
    
    it->p = this;
    it->setUnseen();
  }
  
}



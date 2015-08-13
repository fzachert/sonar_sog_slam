/* ----------------------------------------------------------------------------
 * particle.cpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a particle of the sog-slam-particle-filter
 * ----------------------------------------------------------------------------
*/

#include "particle.hpp"
#include <base/logging.h>

using namespace sonar_sog_slam;

ModelConfig Particle::model_config = ModelConfig();

SOG_Map Particle::get_map(){
  SOG_Map map;
  map.time = this->time;
  
  for(std::list<ParticleFeature>::iterator it = features.begin(); it != features.end(); it++){
    SOG_Feature sf;    
    Simple_Feature simple_f;
    simple_f.pos = base::Vector3d::Zero();
    
    for( std::list<EKF>::iterator it_ekf = it->gaussians.begin(); it_ekf != it->gaussians.end(); it_ekf++){
      Gaussian g;
      g.mean = it_ekf->state;
      g.cov = it_ekf->cov;
      g.weight = it_ekf->weight;
      sf.gaussians.push_back(g);
      
      simple_f.pos += it_ekf->weight * it_ekf->state;
    }
    
    map.simple_features.push_back( simple_f);
    map.features.push_back(sf);
  } 
  
  return map;
}

void Particle::set_unseen(){
  
  for(std::list<ParticleFeature>::iterator it = features.begin(); it != features.end(); it++){
    
    it->p = this;
    it->set_unseen();
  }
  
}

void Particle::reduce_features(){
  
  if(features.size() < 2)
    return;
  
  for(std::list<ParticleFeature>::iterator it = features.begin(); it != features.end(); it++){
    
    for(std::list<ParticleFeature>::iterator it_ = features.begin(); it_ != features.end();){
      
      if( it != it_){
      
	if( (it->average_state - it_->average_state).norm() < model_config.merge_distance_threshold){
	
	  it->merge( *it_);
	  it_ = features.erase(it_);
	  continue;
	}
	
      }
      
      it_++;
      
    }    
  }
  
}



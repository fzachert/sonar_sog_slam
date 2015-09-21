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

SOG_Map Particle::get_map( base::Vector3d transformation){
  SOG_Map map;
  map.time = this->time;
  
  for(std::list<ParticleFeature>::iterator it = features.begin(); it != features.end(); it++){
    SOG_Feature sf;    
    Simple_Feature simple_f;
    simple_f.pos = it->average_state + transformation;
    simple_f.descriptor = 1;
    
    for( std::list<EKF>::iterator it_ekf = it->gaussians.begin(); it_ekf != it->gaussians.end(); it_ekf++){
      Gaussian g;
      g.mean = it_ekf->state + transformation;
      g.cov = it_ekf->cov;
      g.weight = it_ekf->weight;
      sf.gaussians.push_back(g);
      
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

void Particle::reduce_features( double merge_distance_threshold){
  
  if(features.size() < 2)
    return;
  
  double feature_distance;
  
  for(std::list<ParticleFeature>::iterator it = features.begin(); it != features.end(); it++){
    
    for(std::list<ParticleFeature>::iterator it_ = it; it_ != features.end();){
      
      if( it != it_ && (it->changed || it_->changed) ){      
	
	feature_distance = (it->average_state - it_->average_state).norm();
	
	if( feature_distance < merge_distance_threshold){
	
	  it->merge( *it_);
	  it->reduced = false;
	  it_ = features.erase(it_);
	  continue;
	}
	else if( feature_distance < merge_distance_threshold * 2.0){
	  
	  bool merge = false;
	  
	  for(std::list<EKF>::iterator it_ekf = it->gaussians.begin(); it_ekf != it->gaussians.end() && (!merge) ; it_ekf++){
	    for(std::list<EKF>::iterator it_ekf_ = it_->gaussians.begin(); it_ekf_ != it_->gaussians.end() && (!merge); it_ekf_++){
	      
	      if( (it_ekf->state - it_ekf_->state).norm() < merge_distance_threshold){
		merge = true;
	      }
	      
	    }
	  }
	  
	  it->merge( *it_);
	  it->reduced = false;
	  it_ = features.erase(it_);
	  continue;
	  
	}
	
      }
      
      it_++;
      
    }
    
    it->changed = false;
  }
  
}



/* ----------------------------------------------------------------------------
 * candidate_filter.cpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a candidate filter, which rejects outliers on a particle-basis
 * ----------------------------------------------------------------------------
*/

#include "candidate_filter.hpp"
#include "../slam/particle.hpp"
#include <base/logging.h>

using namespace sonar_sog_slam;

void CandidateFilter::init(ModelConfig config){
  
  candidates.clear();
  this->config = config;
  
  
}
      
bool CandidateFilter::check_candidate( double range, double alpha_h, Particle *p){
  
  LOG_DEBUG_S << "Check candidate with range " << range << " and alpha " << alpha_h
    << " and " << candidates.size() << " candidates"; 
  
  base::Vector3d pos;
  double plane_range = range * cos( base::getPitch(p->ori) + config.sonar_vertical_angle);
  double yaw = base::getYaw( p->ori) + alpha_h;
  pos.x() = p->pos.x() + ( plane_range * std::cos( yaw) );  
  pos.y() = p->pos.y() + ( plane_range * std::sin( yaw) );
  pos.z() = 0.0;
  
  double min_dist = config.candidate_distance;
  std::list<Candidate>::iterator min_it = candidates.end();
  
  for(std::list<Candidate>::iterator it = candidates.begin(); it != candidates.end(); it++){
      
    if(it->seen)
      continue;
    
    double dist = ( it->pos - pos).norm();
    
    if(dist < min_dist){
      min_dist = dist;
      min_it = it;
    }
    
  }
  
  if(min_it != candidates.end()){
    min_it->seen = true;
    min_it->confidence++;
    min_it->pos = pos;
    
    LOG_DEBUG_S << "Found matching candidate";
    
    if(min_it->confidence >= config.candidate_threshold){
      LOG_DEBUG_S << "Erase matching candidate";
      candidates.erase(min_it);
      return true;
    }
    
  }else{
    LOG_DEBUG_S << "Create new candidate";
    Candidate c;
    c.pos = pos;
    c.confidence = 1;
    c.seen = true;
    candidates.push_back(c);
  }
  
  return false;
}

void CandidateFilter::reduce_candidates(){  
  
  LOG_DEBUG_S << "Reduce candidates, with " << candidates.size() << " candidates";
  
  for(std::list<Candidate>::iterator it = candidates.begin(); it != candidates.end(); it++){
    
    if(!it->seen){
      it->confidence--;
      
      if(it->confidence <= 0){
	
	LOG_DEBUG_S << "Erase candidate: " << it->pos.transpose();
	it = candidates.erase(it);
	
	if(it != candidates.begin())
	  it--;
	
      }
      
    }else{
      it->seen = false;
    }
    
  }  
  
}
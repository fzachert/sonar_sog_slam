#include "candidate_filter.hpp"
#include "../slam/particle.hpp"

using namespace sonar_sog_slam;

void CandidateFilter::init(ModelConfig config){
  
  candidates.clear();
  this->config = config;
  
  
}
      
bool CandidateFilter::check_candidate( double range, double alpha_h, Particle *p){
  
  base::Vector3d pos;
  double plane_range = range * cos( base::getPitch(p->ori) + config.sonar_vertical_angle);
  double yaw = base::getYaw( p->ori);
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
    
    if(min_it->confidence >= config.candidate_threshold){
      candidates.erase(min_it);
      return true;
    }
    
  }else{
    Candidate c;
    c.pos = pos;
    c.confidence = 1;
    c.seen = true;
    candidates.push_back(c);
  }
  
  return false;
}

void CandidateFilter::reduce_candidates(){  
  
  for(std::list<Candidate>::iterator it = candidates.begin(); it != candidates.end(); it++){
    
    if(!it->seen){
      it->confidence--;
      
      if(it->confidence <= 0){
	
	it = candidates.erase(it);
	
	if(it != candidates.begin())
	  it--;
	
      }
      
    }else{
      it->seen = false;
    }
    
  }  
  
}
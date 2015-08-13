/* ----------------------------------------------------------------------------
 * sog_slam.cpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file implements the sog-slam, using a particle-filter
 * ----------------------------------------------------------------------------
*/

#include "sog_slam.hpp"
#include <base/logging.h>

using namespace sonar_sog_slam;

SOG_Slam::SOG_Slam(){
  
  StaticSpeedNoise = 0;
  StaticOrientationNoise = 0;
  seed = new boost::minstd_rand( static_cast<uint32_t>(time(0)));
  std::srand (time(NULL));
}

SOG_Slam::~SOG_Slam(){
  
 delete StaticSpeedNoise;
 delete StaticOrientationNoise; 
  
}


void SOG_Slam::init(FilterConfig config, ModelConfig mconfig){ 
  
  LOG_DEBUG_S << "Start initializing sog-slam";
  
  this->config = config;
  this->model_config = mconfig;
  Particle::model_config = mconfig;
  
  delete StaticSpeedNoise;
  delete StaticOrientationNoise;
  
  StaticSpeedNoise = new machine_learning::MultiNormalRandom<2>( *seed, Eigen::Vector2d(0.0, 0.0), config.speed_covariance);
  StaticOrientationNoise = new machine_learning::MultiNormalRandom<1>( *seed, Eigen::Matrix< double , 1 , 1, Eigen::DontAlign>::Zero(), Eigen::Matrix< double, 1, 1, Eigen::DontAlign>::Constant(config.orientation_drift_variance) );
  
  initial_feature_likelihood = 0.01;
  initial_feature_likelihood = calculate_initial_feature_likelihood();
  
  init_particles(config.number_of_particles);  
 
  LOG_DEBUG_S << "Initialized sog-slam" << std::endl;
  
}

void SOG_Slam::init_particles( unsigned int number_of_particles){
  
  particles.clear();
  
  for(int i = 0; i < number_of_particles; i++){
    
    Particle p;
    p.pos = base::Vector3d::Zero(); 
    p.yaw_offset = 0.0;
    p.main_confidence = 1.0 / (double)number_of_particles;
    p.candidate_filter.init( model_config);
    
    particles.push_back(p);
    
  } 
  
}

double SOG_Slam::calculate_initial_feature_likelihood(){

  base::Vector3d real_landmark( 7.0, 0.0, -2);
  base::Vector3d noisy_landmark( 7.0 + config.new_feature_distance, 0.0, -2);
  base::Vector3d noisy_landmark2( 7.0, config.new_feature_distance, -2);
  Particle p;
  ParticleFeature pf;
  
  p.pos = base::Vector3d::Zero();
  p.ori = base::Orientation::Identity();
  Particle::model_config = model_config;
  
  pf.p = &p;
  
  base::Vector3d real_measurement = pf.measurement_model_invisable(real_landmark); 
  base::Vector3d noisy_measurement = pf.measurement_model_invisable(noisy_landmark);
  base::Vector3d noisy_measurement2 = pf.measurement_model_invisable(noisy_landmark2);
  pf.init(real_measurement, model_config.sigmaZ, config.number_of_gaussians, config.K);
  
  return (pf.calc_likelihood( noisy_measurement, model_config.sigmaZ) 
    + pf.calc_likelihood( noisy_measurement2, model_config.sigmaZ)) * 0.5;
  
}


double SOG_Slam::observe_features( const sonar_image_feature_extractor::SonarFeatures &z, double weight){
  
  DummyMap m;
  double neff;
  
  if(config.use_markov){
    neff = observe_markov(z, m, weight);
  }else{
    neff = observe(z, m, weight);
  }
  
  debug.effective_sample_size = neff;
  
  if( neff < config.effective_sample_size_threshold)
  {
    resample();
  }
  
  return neff;
}


void SOG_Slam::dynamic(Particle &X, const base::samples::RigidBodyState &u, const DummyMap &m){
  
  if(!X.time.isNull()){
    
    double dt = u.time.toSeconds() - X.time.toSeconds();
    base::Vector3d noisy_vel = u.velocity;
    noisy_vel.block<2,1>(0,0) += (*StaticSpeedNoise)() * dt;
    X.yaw_offset += (*StaticOrientationNoise)().x() * dt;
    X.ori = ( Eigen::AngleAxisd( X.yaw_offset, base::Vector3d::UnitZ()) * global_orientation) ;
    
    X.pos += X.ori * (noisy_vel * dt);
    X.pos.z() = global_depth;
    X.velocity = noisy_vel;
    
  }
  
  X.time = u.time;
  
}





double SOG_Slam::perception(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z, DummyMap &m){
  
  LOG_DEBUG_S << "Perception for particle " << X.pos.transpose() << " with weight " << X.main_confidence
    << " and " << X.features.size() << " features.";
  
  X.set_unseen();
  X.pos.z() = global_depth;
  X.ori = ( Eigen::AngleAxisd( X.yaw_offset, base::Vector3d::UnitZ()) * global_orientation) ;
  
  return perception_positive( X, z) * perception_negative( X, z);
  
}


double SOG_Slam::perception_positive(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z){
  double prob = 1.0;
  
  
  for(std::vector<sonar_image_feature_extractor::Feature>::const_iterator it_z = z.features.begin(); it_z != z.features.end(); it_z++){
    
    if(it_z->range > model_config.max_range)
      continue;    
    
    base::Vector3d meas( it_z->range, it_z->angle_h, model_config.sonar_vertical_angle); 
        
    bool new_feature = true;
    double max_prob = initial_feature_likelihood;
    std::list<ParticleFeature>::iterator max_map_feature = X.features.end();
    
        
    for( std::list<ParticleFeature>::iterator it_p = X.features.begin(); it_p != X.features.end(); it_p++){
      
       if(it_p->seen)
	 continue;
      
       if( it_p->heuristic(meas, model_config.heuristic_tolerance) ){
       
	double prob_temp = it_p->calc_likelihood( meas, model_config.sigmaZ);
	
	if(prob_temp > max_prob){
	  max_prob = prob;
	  max_map_feature = it_p;	 
	}
	
       }else{
	 continue;
       }
       
    }
    
    if( max_map_feature == X.features.end()){ //Create new feature
      
      if(X.candidate_filter.check_candidate( it_z->range, it_z->angle_h, &X) ){
      
	ParticleFeature pf;
	pf.p = &X;
	pf.init( meas, model_config.sigmaZ, config.number_of_gaussians, config.K);
	
	X.features.push_back(pf);
	
      }
      
    }else{
      
      
      max_map_feature->measurement( meas, model_config.sigmaZ);
      
      if( model_config.reduction_trigger_probability >= 1.0 || double_rand() < model_config.reduction_trigger_probability)
      {
	max_map_feature->reduce_gaussians( model_config.reduction_weight_threshold, model_config.reduction_distance_threshold );
      }
      max_prob = max_map_feature->calc_likelihood( meas, model_config.sigmaZ);
      
      max_map_feature->seen = true;
    }
    
    prob = prob * max_prob;
    
  }
  
  X.candidate_filter.reduce_candidates();
  
  if(model_config.reduction_trigger_probability >= 1.0 || double_rand() < model_config.reduction_trigger_probability)
  {
    X.reduce_features();
  }
    
  return prob;
}

double SOG_Slam::perception_negative(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z){
  double prob = 1.0;
  
  LOG_DEBUG_S << "Negative update with " << X.features.size() << " features.";
  
  for( std::list<ParticleFeature>::iterator it = X.features.begin(); it != X.features.end(); it++){
    
    LOG_DEBUG_S << "Check feature";
    
    if( (! it->seen) && it->is_in_sensor_range()){
      
      LOG_DEBUG_S << "Negative measurement";
      
      prob *= it->negative_update();
      it->reduce_gaussians( model_config.reduction_weight_threshold, model_config.reduction_distance_threshold);
      
      if( it->gaussians.size() == 0){
	
	it = X.features.erase(it);
	LOG_DEBUG_S << "Erase Feature";
	
	if(it != X.features.begin())
	  it--;
	
      }
      
    }    
    
  }
  
  
  return prob;
}

void SOG_Slam::set_depth( const double &depth){
  
  global_depth = depth;
  
}
	  
void SOG_Slam::set_orientation(  const base::Orientation &ori){
  
  global_orientation = ori;
  
}


SOG_Map SOG_Slam::get_map(){
  
  double best_confidence = 0.0;
  std::list<Particle>::iterator best_it;
  
  for(std::list<Particle>::iterator it = particles.begin(); it != particles.end(); it++){
    
    if(it->main_confidence > best_confidence){
      best_confidence = it->main_confidence;
      best_it = it;
    }
    
  }
  
  if( best_confidence > 0.0)
    return best_it->get_map();
  else
    return SOG_Map();
  
}

DebugOutput SOG_Slam::get_debug(){
  
  double sum_features = 0.0;
  double best_confidence = 0.0;
  std::list<Particle>::iterator best_it;
  double yaw_offset_sum = 0.0;
  double abs_offset;
  
  debug.yaw_offset_max = 0.0;
  
  
  for(std::list<Particle>::iterator it = particles.begin(); it != particles.end(); it++){
    
    if(it->main_confidence > best_confidence){
      best_confidence = it->main_confidence;
      best_it = it;
    }
    
    
    abs_offset = std::fabs(it->yaw_offset);
    if( abs_offset > debug.yaw_offset_max){
      debug.yaw_offset_max = abs_offset;
    }
    
    yaw_offset_sum += abs_offset;
    sum_features += it->features.size();
    
  }
  
  debug.yaw_offset_avg = yaw_offset_sum / particles.size();  
  
  if(best_confidence > 0.0){
    debug.avg_number_of_features = sum_features / (double)particles.size();
    debug.best_number_of_features = best_it->features.size();
    
    debug.best_max_number_of_gaussians = 0;
    debug.best_avg_number_of_gaussians = 0.0;
    int size;
    
    for(std::list<ParticleFeature>::iterator it_ekf = best_it->features.begin(); it_ekf != best_it->features.end(); it_ekf++){
      
      size = it_ekf->gaussians.size();
      debug.best_avg_number_of_gaussians += size;
      
      if(debug.best_max_number_of_gaussians < size)
	debug.best_max_number_of_gaussians = size;
      
    }
    
    debug.best_avg_number_of_gaussians /= best_it->features.size();
    
  }
  
  
  debug.initial_feature_likelihood = initial_feature_likelihood;
  
  return debug;
}

double SOG_Slam::double_rand()
{
    return (double)std::rand() / RAND_MAX;
}





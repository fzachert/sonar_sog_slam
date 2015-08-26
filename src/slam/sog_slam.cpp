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
  
  feenableexcept(FE_INVALID | FE_OVERFLOW); //Enable nan-exceptions
  
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
  ParticleFeature::model_config = mconfig;
  
  delete StaticSpeedNoise;
  delete StaticOrientationNoise;
  
  StaticSpeedNoise = new machine_learning::MultiNormalRandom<2>( *seed, Eigen::Vector2d(0.0, 0.0), model_config.speed_covariance);
  StaticOrientationNoise = new machine_learning::MultiNormalRandom<1>( *seed, Eigen::Matrix< double , 1 , 1, Eigen::DontAlign>::Zero(), Eigen::Matrix< double, 1, 1, Eigen::DontAlign>::Constant(model_config.orientation_drift_variance) );
  
  initial_feature_likelihood = 0.01;
  initial_feature_likelihood = calculate_initial_feature_likelihood();
//  std::cout << "Set initial feature likelihood: " << initial_feature_likelihood << std::endl;
  
  init_particles(config.number_of_particles);
  debug.resample_count = 0;
  debug.avg_observations_before_resampling = 0.0;
  observation_count = 0;  
 
  debug.observation_count = 0;
  debug.max_observation_time = 0.0;
  debug.avg_observation_time = 0.0;
  
  global_depth = 0.0;
  global_orientation = base::Orientation::Identity();
  global_time = base::Time::fromSeconds(0.0);
  
  dead_reackoning_state.position = base::Vector3d::Zero();
  
  LOG_DEBUG_S << "Initialized sog-slam" << std::endl;
  
}

void SOG_Slam::init_particles( unsigned int number_of_particles){
  
  particles.clear();
  
  for(int i = 0; i < number_of_particles; i++){
    
    Particle p;
    p.pos = base::Vector3d::Zero(); 
    p.yaw_offset = 0.0;
    p.main_confidence = 1.0 / (double)number_of_particles;
    p.candidate_filter.init( config, model_config);
    
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
  p.velocity = base::Vector3d::Zero();
  p.ori = base::Orientation::Identity();
  ParticleFeature::model_config = model_config;
  
  pf.p = &p;
  
  base::Vector3d real_measurement = pf.measurement_model_invisable(real_landmark); 
  base::Vector3d noisy_measurement = pf.measurement_model_invisable(noisy_landmark);
  base::Vector3d noisy_measurement2 = pf.measurement_model_invisable(noisy_landmark2);
  pf.init(real_measurement, model_config.sigmaZ, config.number_of_gaussians, config.candidate_threshold, config.K);
  
  double likelihood = pf.calc_likelihood( noisy_measurement, model_config.sigmaZ);
  double likelihood2 = pf.calc_likelihood( noisy_measurement2, model_config.sigmaZ);
  
//   std::cout << "Calculate initial feature likelihood: " << std::endl;
//   std::cout << "Real simulated measurement: " << real_measurement.transpose() << std::endl;
//   std::cout << "Noisy simulated measurement: " << noisy_measurement.transpose() << std::endl;  
//   std::cout << "Likelihood1: " << likelihood << std::endl;
//   std::cout << "Likelihood2: " << likelihood2 << std::endl;
  
  return (likelihood + likelihood2) * 0.5;
  
}


double SOG_Slam::observe_features( const sonar_image_feature_extractor::SonarFeatures &z, double weight){
  
  base::Time start_observation = base::Time::now();
  
  DummyMap m;
  double neff;
  
  debug.observation_count++;
  
  if(config.use_markov){
    neff = observe_markov(z, m, weight);
  }else{
    neff = observe(z, m, weight);
  }
  
  debug.effective_sample_size = neff;
  observation_count++;
  
  if( neff < config.effective_sample_size_threshold && observation_count >= config.min_observations)
  {
    resample();
    debug.resample_count++;
    debug.avg_observations_before_resampling += ( 1.0 / (double)debug.resample_count)
      * ( ((double)observation_count) - debug.avg_observations_before_resampling );
    observation_count = 0;  
  }
  
  double observation_time = base::Time::now().toSeconds() - start_observation.toSeconds();
 
  if(observation_time > debug.max_observation_time)
    debug.max_observation_time = observation_time;
  
  debug.avg_observation_time += (1.0 / (double)debug.observation_count)
    * ( ((double)observation_time) - debug.avg_observation_time );
  
  return neff;
}


void SOG_Slam::dynamic(Particle &X, const base::samples::RigidBodyState &u, const DummyMap &m){
  
  if(!X.time.isNull()){
    
    double dt = u.time.toSeconds() - X.time.toSeconds();
    base::Vector3d noisy_vel = u.velocity;
    noisy_vel.block<2,1>(0,0) += (*StaticSpeedNoise)() * dt;
    X.yaw_offset += (*StaticOrientationNoise)().x() * dt;
    X.ori = ( Eigen::AngleAxisd( X.yaw_offset, base::Vector3d::UnitZ()) * global_orientation) ;
    
    X.pos += X.ori * (model_config.velocity_rotation * (noisy_vel * dt));
    X.pos.z() = global_depth;
    X.velocity = noisy_vel;
    
  }
  
  X.time = u.time;
  
}


void SOG_Slam::update_dead_reackoning( const base::samples::RigidBodyState &velocity){
  
  if(!dead_reackoning_state.time.isNull()){
    double dt = velocity.time.toSeconds() - dead_reackoning_state.time.toSeconds();
    dead_reackoning_state.position += global_orientation * (model_config.velocity_rotation * (velocity.velocity *dt) );
    dead_reackoning_state.position.z() = global_depth;
    dead_reackoning_state.velocity = velocity.velocity;
    dead_reackoning_state.orientation = global_orientation;
    
  }
  
  dead_reackoning_state.time = velocity.time;
  
}

base::samples::RigidBodyState SOG_Slam::estimate_dead_reackoning(){
  return dead_reackoning_state;
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
      
       if( it_p->heuristic(meas, config.heuristic_tolerance) ){
       
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
	pf.init( meas, model_config.sigmaZ, config.number_of_gaussians, config.candidate_threshold, config.K);
	
	X.features.push_back(pf);
	
      }
      
    }else{
      
      
      max_map_feature->measurement( meas, model_config.sigmaZ);
      
      if( config.reduction_trigger_probability >= 1.0 || double_rand() < config.reduction_trigger_probability)
      {
	max_map_feature->reduce_gaussians( config.reduction_weight_threshold, config.reduction_distance_threshold );

      }
      max_prob = max_map_feature->calc_likelihood( meas, model_config.sigmaZ);
      
      max_map_feature->seen = true;
    }
    
    prob = prob * max_prob;
    
  }
  
  X.candidate_filter.reduce_candidates();
  
  if(config.reduction_trigger_probability >= 1.0 || double_rand() < config.reduction_trigger_probability)
  {
    X.reduce_features( config.merge_distance_threshold);
  }
    
  return prob;
}

double SOG_Slam::perception_negative(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z){
  double prob = 1.0;
  
  LOG_DEBUG_S << "Negative update with " << X.features.size() << " features.";
  
  for( std::list<ParticleFeature>::iterator it = X.features.begin(); it != X.features.end();){
    
    LOG_DEBUG_S << "Check feature";
    
    if( (! it->seen) && it->is_in_sensor_range()){
      
      LOG_DEBUG_S << "Negative measurement";
      
      prob *= it->negative_update();
      it->reduce_gaussians( config.reduction_weight_threshold, config.reduction_distance_threshold);
      
      if( it->gaussians.size() == 0){
	
	it = X.features.erase(it);
	LOG_DEBUG_S << "Erase Feature";
	
	continue;
	
      }
      
    }    
    
    it++;
  }
  
  
  return prob;
}

void SOG_Slam::set_depth( const double &depth){
  
  global_depth = depth;
  
}
	  
void SOG_Slam::set_orientation(  const base::Orientation &ori){
  
  global_orientation = ori;
  
}

void SOG_Slam::set_time( const base::Time &time){
  global_time = time;
}


SOG_Map SOG_Slam::get_map(base::Vector3d transformation){
  
  double best_confidence = 0.0;
  std::list<Particle>::iterator best_it;
  
  for(std::list<Particle>::iterator it = particles.begin(); it != particles.end(); it++){
    
    if(it->main_confidence > best_confidence){
      best_confidence = it->main_confidence;
      best_it = it;
    }
    
  }
  
  if( best_confidence > 0.0)
    return best_it->get_map(transformation);
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
  
  debug.max_number_of_candidates = 0;
  debug.avg_number_of_candidates = 0.0;
  
  for(std::list<Particle>::iterator it = particles.begin(); it != particles.end(); it++){
    
    if(it->main_confidence > best_confidence){
      best_confidence = it->main_confidence;
      best_it = it;
    }
    
    
    abs_offset = std::fabs(it->yaw_offset);
    if( abs_offset > debug.yaw_offset_max){
      debug.yaw_offset_max = abs_offset;
    }
    
    if(it->candidate_filter.candidates.size() > debug.max_number_of_candidates)
      debug.max_number_of_candidates = it->candidate_filter.candidates.size();
    
    debug.avg_number_of_candidates += it->candidate_filter.candidates.size();
    
    
    yaw_offset_sum += abs_offset;
    sum_features += it->features.size();
    
  }
  
  debug.yaw_offset_avg = yaw_offset_sum / particles.size(); 
  debug.avg_number_of_candidates /= particles.size();
  
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
    
    if(best_it->features.size() > 0)
      debug.best_avg_number_of_gaussians /= best_it->features.size();
    else
      debug.best_avg_number_of_gaussians = 0.0;
    
  }  
  
  debug.initial_feature_likelihood = initial_feature_likelihood;
  
  return debug;
}

double SOG_Slam::double_rand()
{
    return (double)std::rand() / RAND_MAX;
}





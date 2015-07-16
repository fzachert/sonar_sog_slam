#include "sog_slam.hpp"

using namespace sonar_sog_slam;

SOG_Slam::SOG_Slam(){
  
  StaticSpeedNoise = 0;
  StaticOrientationNoise = 0;
}

SOG_Slam::~SOG_Slam(){
  
 delete StaticSpeedNoise;
 delete StaticOrientationNoise; 
  
}


void SOG_Slam::init(FilterConfig config, ModelConfig mconfig){ 
  
  this->config = config;
  this->model_config = mconfig;
  Particle::model_config = mconfig;
  
  delete StaticSpeedNoise;
  delete StaticOrientationNoise;
  boost::minstd_rand seed(static_cast<uint32_t>(time(0))); //TODO
  
  StaticSpeedNoise = new machine_learning::MultiNormalRandom<2>(seed, Eigen::Vector2d(0.0, 0.0), config.speed_covariance);
  StaticOrientationNoise = new machine_learning::MultiNormalRandom<1>( seed, Eigen::Matrix< double , 1 , 1>::Zero(), Eigen::Matrix< double, 1,1>(config.orientation_drift_variance) );

  init_particles(config.number_of_particles);
 
}

void SOG_Slam::init_particles( unsigned int number_of_particles){
  
  particles.clear();
  
  for(int i = 0; i < number_of_particles; i++){
    
    Particle p;
    p.pos = base::Vector3d::Zero(); 
    p.yaw_offset = 0.0;
    p.main_confidence = 1.0 / (double)number_of_particles;
    
    particles.push_back(p);
    
  } 
  
}


double SOG_Slam::observeFeatures( const sonar_image_feature_extractor::SonarFeatures &z, double weight){
  
  DummyMap m;
  if(config.use_markov){
    return observe_markov(z, m, weight);
  }else{
    return observe(z, m, weight);
  }
  
}


void SOG_Slam::dynamic(Particle &X, const base::samples::RigidBodyState &u, DummyMap &m){
  
  if(!last_velocity_sample.isNull()){
    
    double dt = u.time.toSeconds() - last_velocity_sample.toSeconds();
    base::Vector3d noisy_vel = u.velocity;
    noisy_vel.block<2,1>(0,0) += (*StaticSpeedNoise)() * dt;
    X.yaw_offset += (*StaticOrientationNoise)().x() * dt;
    X.ori = ( Eigen::AngleAxisd( X.yaw_offset, base::Vector3d::UnitZ()) * X.global_orientation) ;
    
    X.pos += X.ori * (noisy_vel * dt);
    X.velocity = noisy_vel;
    
  }
  
   last_velocity_sample = u.time;
  
}





double SOG_Slam::perception(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z, DummyMap &m){
  
  
  return perception_positive( X, z) * perception_negative( X, z);
  
}


double SOG_Slam::perception_positive(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z){
  double prob = 1.0;
  
  for(std::vector<sonar_image_feature_extractor::Feature>::const_iterator it_z = z.features.begin(); it_z != z.features.end(); it_z++){
    
    base::Vector3d meas( it_z->range, it_z->angle_h, model_config.sonar_vertical_angle); 
        
    bool new_feature = true;
    double max_prob = config.new_feature_probability;
    std::list<ParticleFeature>::iterator max_map_feature = X.features.end();
    
        
    for( std::list<ParticleFeature>::iterator it_p = X.features.begin(); it_p != X.features.end(); it_z++){
      
       double prob_temp = it_p->calcLikelihood( meas, config.sigmaZ);
      
       if(prob_temp > max_prob){
	 max_prob = prob;
	 max_map_feature = it_p;	 
       }       
       
    }
    
    if( max_map_feature == X.features.end()){ //Create new feature
      
      ParticleFeature pf;
      
      pf.init( meas, config.sigmaZ, config.number_of_gaussians, config.K);
      
      X.features.push_back(pf);
      
    }else{
      
      
      max_map_feature->measurement( meas, config.sigmaZ);
      
      max_prob = max_map_feature->calcLikelihood( meas, config.sigmaZ);
    }
    
    prob = prob * max_prob;
    
  }
  
  return prob;
}

double SOG_Slam::perception_negative(Particle &X, const sonar_image_feature_extractor::SonarFeatures &z){
  double prob = 1.0;
  
  for( std::list<ParticleFeature>::iterator it = X.features.begin(); it != X.features.end(); it++){
    
    if( ! it->seen && it->is_in_sensor_range()){
      
//      X.negativeUpdate(z);
      
    }
    
    
  }
  
  
  return prob;
}






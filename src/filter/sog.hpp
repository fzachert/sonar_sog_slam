#ifndef _SOGSSLAM_SOG_HPP_
#define _SOGSSLAM_SOG_HPP_

#include <base/Eigen.hpp>
#include "ekf.hpp"
#include <list>
#include <math.h>

namespace sonar_sog_slam
{
  
    template <unsigned int MODEL_DIM1, unsigned int MODEL_DIM2>
    class SOG
    {
	public:
	  std::list<EKF> gaussians;
      
	protected:
	  
          

	  void updateWeights( Eigen::Matrix<double, MODEL_DIM2, 1> z, Eigen::Matrix<double, MODEL_DIM2, MODEL_DIM2> sigmaZ ){
	    
	    for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
	      
	      if( isVisable( it->state) ){
		
		double likelihood = it->calcLikelihood<MODEL_DIM1>(z.template block<MODEL_DIM1,1>(0,0), measurement_model_visable( it->state),
		  jacobi_measurement_model_visable( it->state ), sigmaZ.template block<MODEL_DIM1, MODEL_DIM1>(0,0));
		
		it->weight = it->weight * likelihood;
		
		
	      }else{
		
		double likelihood = it->calcLikelihood<MODEL_DIM2>(z, measurement_model_invisable( it->state),
		  jacobi_measurement_model_invisable( it->state ), sigmaZ);
		
		it->weight = it->weight * likelihood;		
		
		
	      }
	      	      
	      
	    }
	    
	    normalizeWeights();
	    
	  }
	  

	    void normalizeWeights(){
	      
	      double sum = 0;
	      
	      for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		
		sum += it->weight;   
		
	      }
	      
	      for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		
		it->weight = it->weight/sum;   
		
	      }
	      
	    }	  
	  
	  
	  
	  
        public: 
    
	    
	    double calcLikelihood( Eigen::Matrix<double, MODEL_DIM2, 1> z, Eigen::Matrix<double, MODEL_DIM2, MODEL_DIM2> sigmaZ){
	      	
	      double likelihood = 0.0;
	      
	      for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		
		if( isVisable( it->state) ){
		  
		  likelihood += it->weight * it->calcLikelihood<MODEL_DIM1>(z.template block<MODEL_DIM1, 1>(0,0), measurement_model_visable(it->state),
		    jacobi_measurement_model_visable( it->state ), sigmaZ.template block<MODEL_DIM1, MODEL_DIM1>(0,0) );			
		  
		}else{
		  
		  likelihood += it->weight * it->calcLikelihood<MODEL_DIM2>(z, measurement_model_invisable(it->state),
		    jacobi_measurement_model_invisable( it->state ), sigmaZ);		
		  
		}
			
		
	      }
	      
	      return likelihood;
	    }	    

	    
	    void measurement( Eigen::Matrix<double, MODEL_DIM2, 1> z, Eigen::Matrix<double, MODEL_DIM2, MODEL_DIM2> sigmaZ){
	      
	      
	      for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		
		if( isVisable( it->state) ){
		  
		  it->measurement<MODEL_DIM1>(  z.template block<MODEL_DIM1, 1>(0,0), measurement_model_visable(it->state),
		    sigmaZ.template block<MODEL_DIM1, MODEL_DIM1>(0,0), jacobi_measurement_model_visable( it->state) );
	  
		  
		}else{

		  it->measurement<MODEL_DIM2>( z, measurement_model_invisable( it->state),
		    sigmaZ, jacobi_measurement_model_invisable( it->state) );
		  		  
		}
		it->counter++;
	      }	
	      
	      updateWeights(z, sigmaZ);
	      
	    }


	    void reduceGaussians( double weight_threshold, double distance_threshold){
	      
	      if(gaussians.size() < 2)
		return;
	      
	      std::list<EKF> new_gaussians;
	      std::list<EKF> temp;
	      
	      for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		
		if(it->weight < weight_threshold){
		  it = gaussians.erase(it);
		  
		  if(it!= gaussians.begin())
		    it--;
		  
		}    
	      }  
	      

	      
	      while(gaussians.size() > 0){
	      
		EKF max_EKF;
		double max_weight = 0.0;
		
		for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		  
		  if(it->weight > max_weight){
		    
		    max_EKF = *it;
		    max_weight = it->weight;
		  }    
		  
		}
		
		temp.clear(); 
		double sum_weights = 0;
		base::Vector3d sum_means = base::Vector3d::Zero();
		base::Matrix3d sum_cov = base::Matrix3d::Zero();    
		
		for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		  
		  double dist = std::sqrt((max_EKF.state - it->state).transpose() * max_EKF.cov.inverse() * (max_EKF.state - it->state) );
		  
		  if(dist < distance_threshold){
		    temp.push_back(*it);
		    
		    sum_weights += it->weight;	
		    sum_means += (it->weight * it->state);	
		    
		    it = gaussians.erase(it);
		    
		    if(it != gaussians.begin())
		      it--;
		    
		  }
		  
		}
		
		EKF new_EKF;
		new_EKF.weight = sum_weights;
		new_EKF.state = sum_means / sum_weights;
		
		for(std::list<EKF>::iterator it = temp.begin(); it != temp.end(); it++){
		  
		  sum_cov += it->weight * ( it->cov + ( (new_EKF.state - it->state )*((new_EKF.state - it->state ).transpose())   ) );
		  
		}  
		
		new_EKF.cov = sum_cov / sum_weights;
		new_gaussians.push_back(new_EKF);
	      }
	      
	      gaussians = new_gaussians;
	      
	    }


  
	    virtual void init(Eigen::Matrix<double, MODEL_DIM2, 1> z, Eigen::Matrix<double, MODEL_DIM2, MODEL_DIM2> cov_z, int number_of_gaussians, double K = 0.4) = 0;
	    	    
	    virtual Eigen::Matrix<double, MODEL_DIM1, 1> measurement_model_visable( base::Vector3d landmark) = 0;
	    
	    virtual Eigen::Matrix<double, MODEL_DIM2, 1> measurement_model_invisable( base::Vector3d landmark) = 0;
	    
	    virtual Eigen::Matrix<double, MODEL_DIM1, 3> jacobi_measurement_model_visable( base::Vector3d landmark) = 0;
	    
	    virtual Eigen::Matrix<double, MODEL_DIM2, 3> jacobi_measurement_model_invisable( base::Vector3d landmark) = 0;	    
	    
	    virtual bool isVisable( base::Vector3d landmark) = 0;

    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
/* ----------------------------------------------------------------------------
 * sog.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides a generic sum-of-gaussian-filter
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_SOG_HPP_
#define _SOGSSLAM_SOG_HPP_

#include <base/Eigen.hpp>
#include "ekf.hpp"
#include <list>
#include <math.h>

namespace sonar_sog_slam
{
  
    /**
     * Generic sum-of-gaussian-filter, with measurement with MODEL_DIM1 and MODEL_DIM2 dimensions
     * MODEL_DIM1 is used if the state is visable, otherwise MODEL_DIM2 is used
     * The measurement-model needs to be implemented in a child-class
     */
    template <unsigned int MODEL_DIM1, unsigned int MODEL_DIM2>
    class SOG
    {
	public:
	  std::list<EKF> gaussians;
	    
	  base::Vector3d average_state;
	  
	  bool changed;
	  bool reduced;
	  
	protected:
	  
          
/**
 * Update the weights of the EKFs, based on the likelihood to one measurement
 * @param z: Measurement
 * @param sigmaZ: covaraince of the measurement
 */
	  void update_weights( const Eigen::Matrix<double, MODEL_DIM2, 1> &z, const Eigen::Matrix<double, MODEL_DIM2, MODEL_DIM2> &sigmaZ ){
	    
	    for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
	      
	      if( is_visable( it->state) ){
		
		double likelihood = it->calc_likelihood<MODEL_DIM1>(z.template block<MODEL_DIM1,1>(0,0), measurement_model_visable( it->state),
		  jacobi_measurement_model_visable( it->state ), sigmaZ.template block<MODEL_DIM1, MODEL_DIM1>(0,0));
		
		it->weight = it->weight * likelihood;
		
		
	      }else{
		
		double likelihood = it->calc_likelihood<MODEL_DIM2>(z, measurement_model_invisable( it->state),
		  jacobi_measurement_model_invisable( it->state ), sigmaZ);
		
		it->weight = it->weight * likelihood;		
		
		
	      }
	      	      
	      
	    }
	    
	    normalize_weights();
	    update_average_state();
	  }
	  
/**
 * Normalize the weight of the EKFs
 */
	    void normalize_weights(){
	      
	      double sum = 0;
	      
	      for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		
		sum += it->weight;   
		
	      }
	      
	      if(sum > 0){
		for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		  
		  it->weight = it->weight/sum;   
		  
		}
	      }else{
		 for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		  
		  it->weight = 1.0 / (double)gaussians.size();   
		  
		}
		
	      }
	      
	    }	  
	  
	  
/**
* Update the average state, as a weighted average of the EKFs
*/ 
	  
	    void update_average_state(){
	      
	      if(gaussians.size() == 1){
		average_state = gaussians.begin()->state;
		return;
	      }
	      
	      average_state = base::Vector3d::Zero();
	      
	      for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		  average_state += it->weight * it->state;
	      }
	      
	    }
	  
	  
        public:    
	   
	  
	  /**
	   * Calculate the likelihood of a measurement
	   * @param z: measurement
	   * @param sigmaZ: covariance of measurement
	   * @return: likelihood of the measurement
	   */
	   double calc_likelihood( const Eigen::Matrix<double, MODEL_DIM2, 1> &z, const Eigen::Matrix<double, MODEL_DIM2, MODEL_DIM2> &sigmaZ){
	      	
	      double likelihood = 0.0;
	      
	      for( std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		
		if( is_visable( it->state) ){
		  
		  likelihood += it->weight * it->calc_likelihood<MODEL_DIM1>(z.template block<MODEL_DIM1, 1>(0,0), measurement_model_visable(it->state),
		    jacobi_measurement_model_visable( it->state ), sigmaZ.template block<MODEL_DIM1, MODEL_DIM1>(0,0) );			
		  
		}else{
		  
		  likelihood += it->weight * it->calc_likelihood<MODEL_DIM2>(z, measurement_model_invisable(it->state),
		    jacobi_measurement_model_invisable( it->state ), sigmaZ);		
		  
		}
			
		
	      }
	      
	      return likelihood;
	    }	    

	    /**
	     * Measurement-update of the SOG
	     * @param z: Measurement
	     * @param sigmaZ: Covaraince of the measurement
	     */
	    void measurement( const Eigen::Matrix<double, MODEL_DIM2, 1> &z, const Eigen::Matrix<double, MODEL_DIM2, MODEL_DIM2> &sigmaZ){
	      
	      
	      for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
		
		if( is_visable( it->state) ){
		  
		  it->measurement<MODEL_DIM1>(  z.template block<MODEL_DIM1, 1>(0,0), measurement_model_visable(it->state),
		    sigmaZ.template block<MODEL_DIM1, MODEL_DIM1>(0,0), jacobi_measurement_model_visable( it->state) );
	  
		  
		}else{

		  it->measurement<MODEL_DIM2>( z, measurement_model_invisable( it->state),
		    sigmaZ, jacobi_measurement_model_invisable( it->state) );
		  		  
		}
		it->counter++;
	      }	
	      
	      update_weights(z, sigmaZ);
	      
	      changed = true;
	      reduced = false;
	    }

	    /**
	     * Reduce the gaussians
	     * Remove gaussians if their with is to small and merge them if they are similar
	     * @param weight_threshold: Gaussians below this threshold will be removed
	     * @param distance_threshold: Gaussian with a distance below ths threshold will be merged
	     */
	    void reduce_gaussians( const double &weight_threshold, const double &distance_threshold){
	      
	      if(gaussians.size() < 2)
		return;
	      
	      std::list<EKF> new_gaussians;
	      std::list<EKF> temp;
	      
	      for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end(); ){
		
		if(it->weight < weight_threshold){
		  it = gaussians.erase(it);
		  
		}else{
		  it++;
		}
	      }  
	      

	      
	      while(gaussians.size() > 0){
	      
		EKF max_EKF = gaussians.front();
		double max_weight = gaussians.begin()->weight;
		
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
		unsigned int max_counter = 0;
		bool is_visable = false;
		
		for(std::list<EKF>::iterator it = gaussians.begin(); it != gaussians.end();){
		  
		  double dist = std::sqrt((max_EKF.state - it->state).transpose() * max_EKF.cov.inverse() * (max_EKF.state - it->state) );
		  
		  if(dist < distance_threshold){
		    temp.push_back(*it);
		    
		    sum_weights += it->weight;	
		    sum_means += (it->weight * it->state);	
		    max_counter = std::max( max_counter, it->counter);
		    
		    if( it->visable)
		      is_visable = true;
		    
		    it = gaussians.erase(it);
		    
		  }else{
		    it++;
		  }
		  
		}
		
		EKF new_EKF;
		new_EKF.weight = sum_weights;
		new_EKF.state = sum_means / sum_weights;
		new_EKF.counter = max_counter;
		new_EKF.visable = is_visable;

		for(std::list<EKF>::iterator it = temp.begin(); it != temp.end(); it++){
		  
		  sum_cov += it->weight * ( it->cov + ( (new_EKF.state - it->state )*((new_EKF.state - it->state ).transpose())   ) );
		  
		}  
		
		new_EKF.cov = sum_cov / sum_weights;
		new_gaussians.push_back(new_EKF);
	      }
	      
	      gaussians = new_gaussians;
	      
	      reduced = true;
	      changed = true;
	    }


	    /*
	     * Merge too SOG-Filter into one, be merge the single gausians
	     */
	    void merge( const SOG<MODEL_DIM1, MODEL_DIM2> &other){
	      
	      gaussians.insert( gaussians.end(), other.gaussians.begin(), other.gaussians.end());
	      
	      normalize_weights();
	      update_average_state();
	      
	    }
	      
	    /**
	     * Init-function of the sog-filter. NEEDS to be implemented
	     * Creates the inital gaussians based on one measurement
	     * @param z: Measurement
	     * @param cov_z: covariance of the measurement
	     * @param number_of_gaussians: Number of initial gaussians
	     * @param K: factor k describes the gaussian-variance ith respect to the distance
	     */
	    virtual void init(const Eigen::Matrix<double, MODEL_DIM2, 1> &z, const Eigen::Matrix<double, MODEL_DIM2, MODEL_DIM2> &cov_z, int number_of_gaussians, int initial_counter, double K = 0.4) = 0;
	    
	    /**
	    * Measurement-model if the state is visable. NEEDS to be implemented
	    * @param landmark: state 
	    * @return Model-measurement
	    */	    
	    virtual Eigen::Matrix<double, MODEL_DIM1, 1> measurement_model_visable( const base::Vector3d &landmark) = 0;
	    
	    
	    /**
	    * Measurement-model if the state is invisable. NEEDS to be implemented
	    * @param landmark: state 
	    * @return Model-measurement
	    */		    
	    virtual Eigen::Matrix<double, MODEL_DIM2, 1> measurement_model_invisable( const base::Vector3d &landmark) = 0;
	    
	    
	    /**
	    * Jacobian of the visable measurement-model. NEEDS to be implemented
	    * @param landmark: state 
	    * @return Jacobian of measurement-model
	    */		    
	    virtual Eigen::Matrix<double, MODEL_DIM1, 3> jacobi_measurement_model_visable( const base::Vector3d &landmark) = 0;
	    
	    /**
	    * Jacobian of the invisable measurement-model. NEEDS to be implemented
	    * @param landmark: state 
	    * @return Jacobian of measurement-model
	    */		    
	    virtual Eigen::Matrix<double, MODEL_DIM2, 3> jacobi_measurement_model_invisable( const base::Vector3d &landmark) = 0;	    
	    
	    /**
	     * Checks if a state is visable. NEEDS to be implemented
	     * @param landmark: state
	     * @return: true, if state is visable
	     */
	    virtual bool is_visable( const base::Vector3d &landmark) = 0;

    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
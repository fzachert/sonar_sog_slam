/* ----------------------------------------------------------------------------
 * ekf.hpp
 * written by Fabio Zachert, August 2015
 * University of Bremen
 * 
 * This file provides aa extended 3d kalmanfilter
 * ----------------------------------------------------------------------------
*/

#ifndef _SOGSSLAM_EKF_HPP_
#define _SOGSSLAM_EKF_HPP_
#include <base/Eigen.hpp>
#include <math.h>
#include <machine_learning/GaussianParameters.hpp>


namespace sonar_sog_slam
{
  
    /**
     * Extended 3d kalmanfilter class
     */
    class EKF
    {
      
    public:
      
	  double weight;
	  base::Vector3d state;
	  base::Matrix3d cov;
          bool visable;
	  unsigned int counter;
	  bool changed = false;
      
        public: 
	    
	  /**
	   * Initialize the kalmanfilter
	   * @param mean: Initial mean-state of the kalmanfilter
	   * @param cov: Initial Covariance-matrix of the kalmanfilter
	   * @param weight: likelihood-weight of the filter, used for sum-of-gaussians filter
	   */ 
	    void init(const base::Vector3d &mean, const base::Matrix3d &cov, double weight);
	    
	    /**
	     * Template measurement-fuction, for measurements with INPUT_SIZE dimensions
	     * @param z: measurement with INPUT_SIZE dimensions
	     * @param h: model-measurement with INPUT_SIZE dimensions
	     * @param sigmaZ: covariance of the masurement as a INPUT_SIZE-dimensional matrix
	     * @param H: Jacobian of the model-function, as a INPUT_SIZEx3 matrix
	     */
	    template <unsigned int INPUT_SIZE>
	    void measurement( const Eigen::Matrix<double, INPUT_SIZE, 1> &z, const Eigen::Matrix<double, INPUT_SIZE, 1> &h, const Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> &sigmaZ, const Eigen::Matrix<double, INPUT_SIZE, 3> &H){
	      
	      Eigen::Matrix<double, INPUT_SIZE, 1> inovation = z - h;
	      Eigen::Matrix<double, 3, INPUT_SIZE> kgain =
		cov * H.transpose() * ((H * cov * H.transpose()) +  sigmaZ).inverse();
	      
		state = state + kgain * inovation;
		cov  = (Eigen::Matrix3d::Identity() - kgain*H)*cov;
		
	    }
	    
	    
	    /**
	     * Template dynamic-fuction, for measurements with INPUT_SIZE dimensions
	     * @param u: new state
	     * @param H_X: jacobian with respect to the state
	     * @param sigmaU: covariance of dynamic-measurement 
	     * @param H_U: jacobian with respect to the dynamic-measurement
	     */
	    template <unsigned int INPUT_SIZE>
	    void dynamic( const Eigen::Vector3d &u, const Eigen::Matrix3d &H_X, const Eigen::Matrix3d &H_U, const Eigen::Matrix3d &sigmaU){
	      
	      state = u;
	      cov = H_U * cov * H_U.transpose() + sigmaU;
	      
	    }
	    
	    /**
	     * Calculates the likelihood to a measurement
	     * @param z: measurement with INPUT_SIZE dimensions
	     * @param h: model-measurement with INPUT_SIZE dimensions
	     * @param sigmaZ: covariance of the masurement as a INPUT_SIZE-dimensional matrix
	     * @param H: Jacobian of the model-function, as a INPUT_SIZEx3 matrix
	     * @return: Likelihood of the measurement
	     */
	    template <unsigned int INPUT_SIZE>
	    double calc_likelihood( const Eigen::Matrix<double, INPUT_SIZE, 1> &z, const Eigen::Matrix<double, INPUT_SIZE, 1> &h, const Eigen::Matrix<double, INPUT_SIZE, 3> &H, const Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> &sigmaZ){     
	      
	      
	      return machine_learning::calc_gaussian_norm<INPUT_SIZE>(h, (H * cov * H.transpose()) + sigmaZ, z) ;
	    }
	    
	    /**
	     * Get the weight of the ekf
	     */
	     double& get_weight(){ return weight; } ;
 
	    /**
	     * Check if the ekf is visable
	     */
	    bool& is_visable(){ return visable;};  
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
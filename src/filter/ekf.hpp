#ifndef _SOGSSLAM_EKF_HPP_
#define _SOGSSLAM_EKF_HPP_
#include <base/Eigen.hpp>
#include <math.h>
#include <machine_learning/GaussianParameters.hpp>


namespace sonar_sog_slam
{
    class EKF
    {
      
    public:
      
	  double weight;
	  base::Vector3d state;
	  base::Matrix3d cov;
          bool visable;
	  unsigned int counter;
      
        public: 
	    
	    void init(base::Vector3d mean, base::Matrix3d cov, double weight);
	    
	    template <unsigned int INPUT_SIZE>
	    void measurement( Eigen::Matrix<double, INPUT_SIZE, 1> z, Eigen::Matrix<double, INPUT_SIZE, 1> h, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> sigmaZ, Eigen::Matrix<double, INPUT_SIZE, 3> H){
	      
	      Eigen::Matrix<double, INPUT_SIZE, 1> inovation = z - h;
	      Eigen::Matrix<double, 3, INPUT_SIZE> kgain =
		cov * H.transpose() * ((H * cov * H.transpose()) +  sigmaZ).inverse();
	      
		state = state + kgain * inovation;
		cov  = (Eigen::Matrix3d::Identity() - kgain*H)*cov;
		
	    }
	    
	    template <unsigned int INPUT_SIZE>
	    void dynamic( Eigen::Vector3d u, Eigen::Matrix3d H_X, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> H_U, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> sigmaU){
	      
	      state = u;
	      cov = H_U * cov * H_U.transpose() + sigmaU;
	      
	    }
	    
	    template <unsigned int INPUT_SIZE>
	    double calcLikelihood( Eigen::Matrix<double, INPUT_SIZE, 1> z, Eigen::Matrix<double, INPUT_SIZE, 1> h, Eigen::Matrix<double, INPUT_SIZE, 3> H, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> sigmaZ){     
	      
	      
	      return machine_learning::calc_gaussian_norm<INPUT_SIZE>(h, (H * cov * H.transpose()) + sigmaZ, z) ;
	    }
	    
	    double& getWeight(){ return weight; } ;
 
	    
	    bool& isVisable(){ return visable;};  
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
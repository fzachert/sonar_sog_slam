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
      
        public: 
            /**
             * Print a welcome to stdout
             * \return nothing
             */
            void welcome();
	    
	    void init(base::Vector3d mean, base::Matrix3d cov, double weight);
	    
	    template <unsigned int INPUT_SIZE>
	    void measurement( Eigen::Matrix<double, INPUT_SIZE, 1> z, Eigen::Matrix<double, INPUT_SIZE, 1> h, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> sigmaZ, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> H){
	      
	      Eigen::Matrix<double, INPUT_SIZE, 1> inovation = z - h;
	      Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> kgain =
		cov * H.transpose * ((H * cov * H.transpose) +  sigmaZ).inverse();
	      
		state = state + kgain * inovation;
		cov  = (Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE>::Identity() - kgain*H)*cov;
		
	    }
	    
	    template <unsigned int INPUT_SIZE>
	    void dynamic( Eigen::Vector3d u, Eigen::Matrix3d H_X, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> H_U, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> sigmaU){
	      
	      state = u;
	      cov = H_U * cov * H_U.transpose() + sigmaU;
	      
	    }
	    
	    template <unsigned int INPUT_SIZE>
	    double calcLikelihood( Eigen::Matrix<double, INPUT_SIZE, 1> z, Eigen::Matrix<double, INPUT_SIZE, INPUT_SIZE> H){
	      	
	      
	      
	      return 0;
	    }
	    
	    double& getWeight(){ return weight; } ;
	    
	    bool isInVerticalAngularRange( double minPitchAngle, double maxPitchAngle);
	    
	    bool isInHorizontalAngularRange( double minYawAngle, double maxYawAngle); 
	    
	    bool& isVisable(){ return visable;};  
	    
    };

} // end namespace sonar_sog_slam

#endif // _DUMMYPROJECT_DUMMY_HPP_
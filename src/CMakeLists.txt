rock_library(sonar_sog_slam
    SOURCES filter/ekf.cpp filter/sog.cpp filter/candidate_filter.cpp 
    slam/sog_slam.cpp slam/particle.cpp  slam/particle_feature.cpp
    HEADERS filter/ekf.hpp filter/sog.hpp filter/candidate_filter.hpp
    slam/sog_slam.hpp slam/particle.hpp  slam/particle_feature.hpp 
    types/slam_particle.hpp types/filter_config.hpp types/model_config.hpp
    types/debug_types.hpp types/dvl_dropout.hpp maps/sog_map.hpp 
    DEPS_PKGCONFIG base-types base-lib sonar_image_feature_extractor )


rock_executable(jacobian_test
    SOURCES test/jacobian_test.cpp
    DEPS_PKGCONFIG sonar_sog_slam base-lib base-types
)  
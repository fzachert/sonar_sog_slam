rock_library(sonar_sog_slam
    SOURCES filter/ekf.cpp filter/sog.cpp slam/sog_slam.cpp
    HEADERS filter/ekf.hpp filter/sog.hpp slam/sog_slam.hpp types/slam_particle.hpp
    DEPS_PKGCONFIG base-types sonar_image_feature_extractor )



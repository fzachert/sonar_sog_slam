rock_vizkit_plugin(sonar_sog_slam-viz
    SOGMapVisualization.cpp ColoredUncertainty.cpp
    DEPS sonar_sog_slam
    MOC SOGMapVisualization.hpp ColoredUncertainty.hpp
    HEADERS SOGMapVisualization.hpp ColoredUncertainty.hpp)


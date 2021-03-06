#ifndef sonar_sog_slam_SOGMapVisualization_H
#define sonar_sog_slam_SOGMapVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <sonar_sog_slam/maps/sog_map.hpp>
#include "ColoredUncertainty.hpp"

namespace vizkit3d
{
    class SOGMapVisualization
        : public vizkit3d::Vizkit3DPlugin<sonar_sog_slam::SOG_Map>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        SOGMapVisualization();
        ~SOGMapVisualization();

    Q_INVOKABLE void updateData(sonar_sog_slam::SOG_Map const &sample)
    {vizkit3d::Vizkit3DPlugin<sonar_sog_slam::SOG_Map>::updateData(sample);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(sonar_sog_slam::SOG_Map const& plan);
        
    private:
        sonar_sog_slam::SOG_Map map;
	std::vector<ColoredUncertainty*> gaussians;
	
	osg::Vec4 featuretype2color( int featuretype);
    };
}
#endif

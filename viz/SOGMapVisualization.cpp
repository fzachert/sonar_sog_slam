#include <iostream>
#include "SOGMapVisualization.hpp"

using namespace vizkit3d;

struct SOGMapVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    sonar_sog_slam::SOG_Map data;
};


SOGMapVisualization::SOGMapVisualization()
    : p(new Data)
{
}

SOGMapVisualization::~SOGMapVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> SOGMapVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void SOGMapVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void SOGMapVisualization::updateDataIntern(sonar_sog_slam::SOG_Map const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(SOGMapVisualization)


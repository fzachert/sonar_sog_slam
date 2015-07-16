#include <iostream>
#include "SOGMapVisualization.hpp"

using namespace vizkit3d;


SOGMapVisualization::SOGMapVisualization()

{
}

SOGMapVisualization::~SOGMapVisualization()
{

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
    
    osg::Group* group = node->asGroup();
    group->removeChildren(0, group->getNumChildren() );
    
    for(std::vector<ColoredUncertainty*>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
      
      delete *it;
    }
    
    gaussians.clear();
      
    int size = 0;
    for(std::vector<sonar_sog_slam::SOG_Feature>::iterator it = map.features.begin(); it != map.features.end(); it++){
      for(std::vector<sonar_sog_slam::Gaussian>::iterator it_gauss = it->gaussians.begin(); it_gauss != it->gaussians.end(); it++){
		
	ColoredUncertainty *cu = new ColoredUncertainty();
	cu->setMean(static_cast<Eigen::Vector3d>( it_gauss->mean) );
	cu->setCovariance( static_cast<Eigen::Matrix3d>( it_gauss->cov)   );
	cu->setColor( osg::Vec4( it_gauss->weight, 1 - it_gauss->weight , 0.0, 1.0 ) );
	gaussians.push_back(cu);
	group->addChild( cu );       
	      
	size++;
      }
      
    }  
    
}

void SOGMapVisualization::updateDataIntern(sonar_sog_slam::SOG_Map const& value)
{
    map = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(SOGMapVisualization)


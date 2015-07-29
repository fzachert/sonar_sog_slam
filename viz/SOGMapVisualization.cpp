#include <iostream>
#include "SOGMapVisualization.hpp"

using namespace vizkit3d;


SOGMapVisualization::SOGMapVisualization()

{
}

SOGMapVisualization::~SOGMapVisualization()
{

    for(std::vector<ColoredUncertainty*>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
      
      delete *it;
    }
    
    gaussians.clear();  
  
  
}

osg::ref_ptr<osg::Node> SOGMapVisualization::createMainNode()
{
    // Geode is a common node used for vizkit3d plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Group();
}

void SOGMapVisualization::updateMainNode ( osg::Node* node )
{
//    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
    
   std::cout << "Update main node with " << map.features.size() << " features" << std::endl;
  
    osg::Group* group = node->asGroup();
    
    std::cout << "Remove children: " << group->getNumChildren() << std::endl;
    group->removeChildren(0, group->getNumChildren() );
    
    std::cout << "Delete uncertaintys" << std::endl;
    for(std::vector<ColoredUncertainty*>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
      
//      delete *it;
    }
    
    gaussians.clear();
    
    std::cout << "Create gaussians" << std::endl;

    for(std::vector<sonar_sog_slam::SOG_Feature>::iterator it = map.features.begin(); it != map.features.end(); it++){
      std::cout << "It feature" << std::endl;
      double d = 0.0;
      for(std::vector<sonar_sog_slam::Gaussian>::iterator it_gauss = it->gaussians.begin(); it_gauss != it->gaussians.end(); it_gauss++){
	std::cout << "It gauss" << std::endl;	
	ColoredUncertainty *cu = new ColoredUncertainty();
	cu->setMean(static_cast<Eigen::Vector3d>( it_gauss->mean) );
	cu->setCovariance( static_cast<Eigen::Matrix3d>( it_gauss->cov)   );
	cu->setColor( osg::Vec4( d, 1 - d , 0.0, 1.0 ) );
	cu->showSamples();
	
	gaussians.push_back(cu);
	group->addChild( cu );       
	
	d += 1.0 / it->gaussians.size();
	std::cout << "Added uncertainty" << std::endl;
      }
      
    }  
    std::cout << "Update end" << std::endl;
}

void SOGMapVisualization::updateDataIntern(sonar_sog_slam::SOG_Map const& value)
{
    map = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(SOGMapVisualization)


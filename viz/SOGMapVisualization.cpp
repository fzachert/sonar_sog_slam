#include <iostream>
#include "SOGMapVisualization.hpp"
#include <base/logging.h>

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
    
   LOG_DEBUG_S << "Update main node with " << map.features.size() << " features";
  
    osg::Group* group = node->asGroup();
    
    group->removeChildren(0, group->getNumChildren() );
    
    for(std::vector<ColoredUncertainty*>::iterator it = gaussians.begin(); it != gaussians.end(); it++){
      
//      delete *it;
    }
    
    gaussians.clear();
    


    for(std::vector<sonar_sog_slam::SOG_Feature>::iterator it = map.features.begin(); it != map.features.end(); it++){

      double d = 0.0;
      for(std::vector<sonar_sog_slam::Gaussian>::iterator it_gauss = it->gaussians.begin(); it_gauss != it->gaussians.end(); it_gauss++){

	ColoredUncertainty *cu = new ColoredUncertainty();
	cu->setColor( osg::Vec4(it_gauss->weight,1.0 - it_gauss->weight, 0.0, 1.0 ) );
	cu->setMean(static_cast<Eigen::Vector3d>( it_gauss->mean) );
	cu->setCovariance( static_cast<Eigen::Matrix3d>( it_gauss->cov)   );	
	cu->showSamples();
	
	gaussians.push_back(cu);
	group->addChild( cu );       
	
	d += 1.0 / it->gaussians.size();

      }
      
    }
    
    for(std::vector<sonar_sog_slam::Simple_Feature>::iterator it = map.simple_features.begin(); it != map.simple_features.end(); it++){
    
      
      osg::Sphere* feature = new osg::Sphere( osg::Vec3( it->pos.x(), it->pos.y(), it->pos.z()), 0.1);
      osg::ShapeDrawable* featureDrawable = new osg::ShapeDrawable(feature);
      featureDrawable->setColor( featuretype2color( it->descriptor ) );
      
      osg::Geode* featureGeode = new osg::Geode();
      featureGeode->addDrawable( featureDrawable);
      
      group->addChild( featureGeode );
      
    }
    

}


osg::Vec4 SOGMapVisualization::featuretype2color( int featuretype){
  
  osg::Vec4 result;
  
  switch(featuretype){
    case 1:
      result = osg::Vec4(1.0 , 1.0, 1.0, 1.0);
      break;
    case 2:
      result = osg::Vec4(1.0, 0.5, 0.5, 1.0);
      break;
    case 3:
      result = osg::Vec4(0.0 ,0.0, 1.0, 1.0);
      break;
    case 4:
      result = osg::Vec4(1.0, 0.0, 0.0, 1.0);
      break;
    
    default:
      result = osg::Vec4(0.0, 0.0, 0.0, 1.0);
  }
  
  return result;
  
}

void SOGMapVisualization::updateDataIntern(sonar_sog_slam::SOG_Map const& value)
{
    map = value;
    LOG_DEBUG_S << "got new sample data";
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(SOGMapVisualization)


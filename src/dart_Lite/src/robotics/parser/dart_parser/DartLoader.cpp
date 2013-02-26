/**
 * @file DartLoader.cpp
 */

#include "DartLoader.h"
#include <map>
#include "../urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
// (Dart-Lite) 
/*
#include "robotics/World.h"
#include "robotics/Robot.h"
#include "robotics/Object.h"
*/

/**
 * @function DartLoader
 * @brief Constructor
 */
DartLoader::DartLoader() {
}

/**
 * @function ~DartLoader
 * @brief Destructor
 */
DartLoader::~DartLoader() {

}

/**
 * @function parseSkeleton
 */
dynamics::SkeletonDynamics* DartLoader::parseSkeleton( std::string _urdfFile ) {

  std::string xml_string;
  xml_string = readXmlToString( _urdfFile );

  boost::shared_ptr<urdf::ModelInterface> skeletonModel = urdf::parseURDF( xml_string );
  
  return modelInterfaceToSkeleton( skeletonModel );

}

/**
 * @function parseRobot
 */
// (Dart-Lite) 
/*
robotics::Robot* DartLoader::parseRobot( std::string _urdfFile ) {

  std::string xml_string;
  xml_string = readXmlToString( _urdfFile );
  boost::shared_ptr<urdf::ModelInterface> robotModel = urdf::parseURDF( xml_string );
  return modelInterfaceToRobot( robotModel );
}
*/

/**
 * @function parseObject
 */
// (Dart-Lite) 
/*
robotics::Object* DartLoader::parseObject( std::string _urdfFile ) {
  
  std::string xml_string;
  xml_string = readXmlToString( _urdfFile );
  boost::shared_ptr<urdf::ModelInterface> objectModel = urdf::parseURDF( xml_string );
  return modelInterfaceToObject( objectModel );

}
*/

/**
 * @function parseWorld
 */
// (Dart-Lite) 
/*
robotics::World* DartLoader::parseWorld( std::string _urdfFile ) {

  mWorldPath = _urdfFile;
 
  // Change path to a Unix-style path if given a Windows one
  // Windows can handle Unix-style paths (apparently)
  std::replace( mWorldPath.begin(), mWorldPath.end(), '\\' , '/' );
  mPath = mWorldPath.substr( 0, mWorldPath.rfind("/") + 1 );

  // std::cout<< " mPath :" << mPath << std::endl;

  robotics::World* world = new robotics::World();
	robotics::Robot* robot;
  robotics::Object* object;

  std::string xml_string;
  xml_string = readXmlToString( _urdfFile );

  boost::shared_ptr<urdf::World> worldInterface =  urdf::parseWorldURDF(xml_string, mPath );

  double roll, pitch, yaw;
  for( unsigned int i = 0; i < worldInterface->objectModels.size(); ++i ) {
    
    object = modelInterfaceToObject(  worldInterface->objectModels[i].model );
    // Initialize position and RPY 
    worldInterface->objectModels[i].origin.rotation.getRPY( roll, pitch, yaw );
    object->setRotationRPY( roll, pitch, yaw );
    
    object->setPositionX( worldInterface->objectModels[i].origin.position.x ); 
    object->setPositionY( worldInterface->objectModels[i].origin.position.y ); 
    object->setPositionZ( worldInterface->objectModels[i].origin.position.z );
    object->update();
    world->addObject( object );
  }
  
  for( unsigned int i = 0; i < worldInterface->robotModels.size(); ++i )  {
    
    robot = modelInterfaceToRobot(  worldInterface->robotModels[i].model );
    // Initialize position and RPY 
    worldInterface->robotModels[i].origin.rotation.getRPY( roll, pitch, yaw );
    robot->setRotationRPY( roll, pitch, yaw );
    
    robot->setPositionX( worldInterface->robotModels[i].origin.position.x ); 
    robot->setPositionY( worldInterface->robotModels[i].origin.position.y ); 
    robot->setPositionZ( worldInterface->robotModels[i].origin.position.z );
    robot->update();
    world->addRobot( robot );
  }
  
  world->rebuildCollision();
  return world;
}
*/

/**
 * @function modelInterfaceToSkeleton
 * @brief Read the ModelInterface and spits out a SkeletonDynamics object
 */
dynamics::SkeletonDynamics* DartLoader::modelInterfaceToSkeleton( boost::shared_ptr<urdf::ModelInterface> _model ) {
  
  dynamics::SkeletonDynamics* mSkeleton; 
  dynamics::BodyNodeDynamics *node, *rootNode;
  kinematics::Joint *joint, *rootJoint;
  
  /** Create new skeleton object */
  mSkeleton = new dynamics::SkeletonDynamics();

  /** Set skeleton name */
  mSkeleton->setName( _model->getName() );

  /** Load links and convert them to DART BodyNodes */
  mNodes.resize(0);  
  
  for( std::map<std::string, boost::shared_ptr<urdf::Link> >::const_iterator lk = _model->links_.begin(); 
       lk != _model->links_.end(); 
       lk++ ) {
    node = createDartNode( (*lk).second, mSkeleton );
    // We return NULL for "world" link, hence this if condition
    if( node != NULL ) { mNodes.push_back( node ); }
  }
  
  if(debug) printf ("** Created %u body nodes \n", mNodes.size() );
  
  /** Initialize Joint store vector */
  mJoints.resize(0);
  
  /** root joint */
  std::string rootName = _model->getRoot()->name;
  rootNode = getNode( rootName );
  
  if(debug) printf ("[DartLoader] Root Node: %s \n", rootNode->getName() );
  
  /** If root node is NULL, nothing to create */
  if( rootNode == NULL ) {
    // Good, we have to set the node attached to world as root
    if( rootName == "world" ) {
      int numRoots = _model->getRoot()->child_links.size();
      if( numRoots != 1 ) { 
	std::cout<< "[ERROR] Not unique link attached to world" <<std::endl; 
      } else {
	rootName = (_model->getRoot()->child_links)[0]->name;
	rootNode = getNode( rootName );
	if( rootNode == NULL ) { return NULL; }
	std::cout<<"[info] World specified in URDF. Root node considered:"<< rootName <<std::endl;
	
	// Since the original rootName was world, add the joint that had it as its parent (only one)
	for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
	     jt != _model->joints_.end(); jt++ ) {  
	  if( ( (*jt).second )->parent_link_name == "world" ) {
	    joint = createDartJoint( (*jt).second, mSkeleton );
	    mJoints.push_back( joint );
	  }
	} // end of else
      } // end of rootName == "world"
    } 
    // Bad. Either the URDF is bad or the structure is not tree-like
    else {
      std::cout << "[ERROR] No root node found!" << std::endl;
      return NULL;
    }
  }
  else {
    /** Create a joint for floating */
    rootJoint =  createDartRootJoint( rootNode, mSkeleton ); 
    mJoints.push_back( rootJoint );
  }   
  
  //-- Save DART structure

  // Push parents first
  std::list<dynamics::BodyNodeDynamics*> nodeStack;
  dynamics::BodyNodeDynamics* u;
  nodeStack.push_back( rootNode );
  
  int numIter = 0;
  while( !nodeStack.empty() && numIter < mNodes.size() ) {
    // Get front element on stack and update it
    u = nodeStack.front();
    // Add it to the Skeleton
    mSkeleton->addNode(u);


    for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
	 jt != _model->joints_.end(); 
	 jt++ ) {  
      if( ( (*jt).second )->parent_link_name == u->getName() ) {
	joint = createDartJoint( (*jt).second, mSkeleton );
	mJoints.push_back( joint );
      }
    }
    
    // Pop it out
    nodeStack.pop_front();
    
    // Add its kids
    for( int idx = 0; idx < u->getNumChildJoints(); ++idx ) {
      nodeStack.push_back( (dynamics::BodyNodeDynamics*)( u->getChildNode(idx) ) );
    }
    numIter++;
  } // end while
  
  if(debug) printf ("[debug] Created %u joints \n", mJoints.size() );
  if(debug) printf ("[debug] Pushed %d nodes in order \n", numIter );
  
  // Init robot (skeleton)
  mSkeleton->initSkel();
  
  return mSkeleton;
}

/**
 * @function modelInterfaceToRobot
 */
// (Dart-Lite) 
/*
robotics::Robot* DartLoader::modelInterfaceToRobot( boost::shared_ptr<urdf::ModelInterface> _model ) {
  
  robotics::Robot* mRobot;
  dynamics::BodyNodeDynamics *node, *rootNode;
  kinematics::Joint *joint, *rootJoint;
  
  // Create new robot object 
  mRobot = new robotics::Robot();
  
  // Set robot name 
  mRobot->setName( _model->getName() );
  
  // Load links and convert them to DART BodyNodes 
  mNodes.resize(0);  

  for( std::map<std::string, boost::shared_ptr<urdf::Link> >::const_iterator lk = _model->links_.begin(); 
       lk != _model->links_.end(); 
       lk++ ) {
    node = createDartNode( (*lk).second, mRobot );
    // We return NULL for "world" link, hence this if condition
    if( node != NULL ) { mNodes.push_back( node ); }
  }

  if(debug) printf ("** Created %u body nodes \n", mNodes.size() );
  
  // Initialize Joint store vector 
  mJoints.resize(0);
  
  // root joint 
  std::string rootName = _model->getRoot()->name;
  rootNode = getNode( rootName );
  
  if(debug) printf ("[DartLoader] Root Node: %s \n", rootNode->getName() );

  // If root node is NULL, nothing to create 
  if( rootNode == NULL ) {
    // Good, we have to set the node attached to world as root
    if( rootName == "world" ) {
      int numRoots = _model->getRoot()->child_links.size();
      if( numRoots != 1 ) { 
	std::cout<< "[ERROR] Not unique link attached to world" <<std::endl; 
      } else {
	rootName = (_model->getRoot()->child_links)[0]->name;
	rootNode = getNode( rootName );
	if( rootNode == NULL ) { return NULL; }
	std::cout<<"[info] World specified in URDF. Root node considered:"<< rootName <<std::endl;

	// Since the original rootName was world, add the joint that had it as its parent (only one)
	for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
	     jt != _model->joints_.end(); jt++ ) {  
	  if( ( (*jt).second )->parent_link_name == "world" ) {
	    joint = createDartJoint( (*jt).second, mRobot );
	    mJoints.push_back( joint );
	  }
	} // end of else
      } // end of rootName == "world"
    } 
    // Bad. Either the URDF is bad or the structure is not tree-like
    else {
      std::cout << "[ERROR] No root node found!" << std::endl;
      return NULL;
    }
  }
  else {
    // Create a joint for floating 
    rootJoint =  createDartRootJoint( rootNode, mRobot ); 
    mJoints.push_back( rootJoint );
  }   
  
  //-- Save DART structure

  // Push parents first
  std::list<dynamics::BodyNodeDynamics*> nodeStack;
  dynamics::BodyNodeDynamics* u;
  nodeStack.push_back( rootNode );
  
  int numIter = 0;
  while( !nodeStack.empty() && numIter < mNodes.size() ) {
    // Get front element on stack and update it
    u = nodeStack.front();
    // Add it to the Robot
    mRobot->addNode(u);

    for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
	 jt != _model->joints_.end(); 
	 jt++ ) {  
      if( ( (*jt).second )->parent_link_name == u->getName() ) {
	joint = createDartJoint( (*jt).second, mRobot );
	mJoints.push_back( joint );
      }
    }
    
    // Pop it out
    nodeStack.pop_front();
    
    // Add its kids
    for( int idx = 0; idx < u->getNumChildJoints(); ++idx ) {
      nodeStack.push_back( (dynamics::BodyNodeDynamics*)( u->getChildNode(idx) ) );
    }
    numIter++;
  } // end while
  
  if(debug) printf ("[debug] Created %u joints \n", mJoints.size() );
  if(debug) printf ("[debug] Pushed %d nodes in order \n", numIter );
  
  // Init robot (skeleton)
  mRobot->initSkel();
  mRobot->update();
  return mRobot;
}
*/

/**
 * @function modelInterfaceToObject
 */
// (Dart-Lite) 
/*
robotics::Object* DartLoader::modelInterfaceToObject( boost::shared_ptr<urdf::ModelInterface> _model ) {
  
  robotics::Object* mObject;
  dynamics::BodyNodeDynamics *node, *rootNode;
  kinematics::Joint *joint, *rootJoint;

  mObject = new robotics::Object();
  
  // name
  mObject->setName( _model->getName() );
  
  // BodyNode
  mNodes.resize(0);  
  for( std::map<std::string, boost::shared_ptr<urdf::Link> >::const_iterator lk = _model->links_.begin(); 
       lk != _model->links_.end(); 
       lk++ ) {
    node = createDartNode( (*lk).second, mObject );
    mNodes.push_back( node );
  }
    if(debug) printf ("[debug] Created %u body nodes \n", mNodes.size() );
  
  // Joint
  mJoints.resize(0);
  
  for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator jt = _model->joints_.begin(); 
       jt != _model->joints_.end(); 
       jt++ ) {  
    joint = createDartJoint( (*jt).second, mObject );
    mJoints.push_back( joint );

  }
  
  //-- root joint
  rootNode = getNode( _model->getRoot()->name ); // no rootnode
  rootJoint = createDartRootJoint( rootNode, mObject );
  mJoints.push_back( rootJoint );
  
  if(debug) printf ("[debug] Created %u joints \n", mJoints.size() );
  
  //-- Save DART structure

  // 1. Root node was added already so add...
  
  // 2. The rest of nodes
  for( unsigned int i = 0; i < mNodes.size(); ++i ) {
    // Add nodes to the object
      mObject->addNode( mNodes[i] );
  }
  
  // Init object (skeleton)
  mObject->initSkel();
  mObject->update();
  
  return mObject;
}
*/

/**
 * @function getNode
 */
dynamics::BodyNodeDynamics* DartLoader::getNode( std::string _nodeName ) {

  for( unsigned int i = 0; i < mNodes.size(); ++i ) {
    std::string node( mNodes[i]->getName() );
    if( node ==  _nodeName ) {
      return mNodes[i];
    }
  }
  if(debug) printf ("[getNode] ERROR: Returning  NULL for  %s \n", _nodeName.c_str() );
  return NULL;
}

/**
 * @function readXml
 */
std::string  DartLoader::readXmlToString( std::string _xmlFile ) {
  
  std::string xml_string;
  
  std::fstream xml_file( _xmlFile.c_str(), std::fstream::in );
  
  // Read xml
  while( xml_file.good() ) {
    std::string line;
    std::getline( xml_file, line );
    xml_string += (line + "\n");
  }
  xml_file.close();
  
  return xml_string;
}

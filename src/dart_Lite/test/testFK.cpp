/**
 * @file testFK.cpp
 * @brief Testing FK accuracy with model
 */

#include "robotics/parser/dart_parser/DartLoader.h"
#include "dynamics/SkeletonDynamics.h"
#include <stdio.h>

/**
 * @function main
 */
int main( int argc, char* argv[] ) {

  printf("Parsing \n");
  // Parsing step
  DartLoader dl;
  dynamics::SkeletonDynamics* hubo;
  hubo = dl.parseSkeleton( argv[1] );
  if( hubo != NULL ) {
    printf("Generated a nice skeleton for Hubo! \n");
  } else {
	printf("Bang! No good skeleton generated! \n");
  }

  return 0;
}

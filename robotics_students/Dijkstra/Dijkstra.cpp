/********************************************************************************
*                                                                            	*
*  Dijkstra.cpp		 		                              	     	*
*  ===================                                                       	*
*                                                                            	*
*  Description: 							     	*
*  It finds the best path in a topological map					*
*  using the Dijkstra algorithm.						*
*									     	*
*  Compile:								     	*
*	make -f Makefile_Dijkstra					     	*
*									     	*
*				Diego Cordero					*
*                               Jesus Savage                                   	*
*                                                                        	*
*                               FI-UNAM 2020                                 	*
********************************************************************************/


#define LARGEST_DISTANCE_SENSORS 0.0

// Include files
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "../utilities/constants.h"
#include "../utilities/structures.h"
#include "../utilities/inputs.h"
#include "dijkstra.h"



//It gets the best nodes using the Dijkstra algorithm 
float Dijkstra(Inputs inputs)
{

 float rx,ry,lx,ly;
 step steps[200];
 char world_file[250];
 float distance;

 sprintf(world_file,"%s%s",inputs.path,inputs.environment);

 rx=inputs.xo;
 ry=inputs.yo;
 //printf("origen %f %f \n",rx,ry);
 lx=inputs.xd;
 ly=inputs.yd;
 //printf("destination %f %f\n",inputs.xd,inputs.yd);

 distance=dijkstra(rx,ry ,lx ,ly,world_file,steps);
	

 return(distance);
}




// Main program
int main(int argc, char *argv[])
{

 Inputs inputs;
 float distance;


 // it gets line inputs, function in ../utilities/inputs.h 
 get_inputs(argc,argv,&inputs);

 //It gets the best nodes using the Dijkstra algorithm 
 distance=Dijkstra(inputs);

 printf("%f\n",distance);

 return 0;

}



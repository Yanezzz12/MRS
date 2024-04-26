/********************************************************************************
*                                                                            	*
*  GoTo_State_Machine.cpp 		                              	     	*
*  ===================                                                       	*
*                                                                            	*
*  Description: 							     	*
*  It controls the movement of a robot using state machines.		     	*
*  It uses a real robot or a simulated one.				     	*
*									     	*
*  Compile:								     	*
*	make -f Makefile_GoTo_State_Machine				     	*
*									     	*
*                               J. Savage                                    	*
*                                                                        	*
*                               FI-UNAM 2015                                 	*
*                               FI-UNAM 2024                                 	*
********************************************************************************/

//Constants
#define DEBUG 1 // Uncomment this line to see debuging data
#define NUM_DELAY_UNK 15
#define TRAINING 1
#define VQ 1
#define CENTROID_FILE "vq_images_laser"
#define NUM_BITS_INPUTS 10
#define NUM_BITS_OUTPUTS 3 // bits to decode 8 outputs: STOP, BACKWARD, FORWARD, TURN_LEFT, TURN_RIGHT, etc
#define NUM_BITS_INTENSITY 2
#define NUM_INTENSITY 4 // 2 ** NUM_BITS_INTENSITY
#define NUM_BITS_DEST_ANGLE 3
#define NUM_DEST_ANGLE 8 // 2 ** NUM_BITS_DEST_ANGLE
#define NUM_MAX_MEMORY 65535 // 2 >> 16
#define FLG_VQ_SRT 1 // it uses a sorted VQ
#define THRS 0.08 // Threshold to detect obstacles


// System include files
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>

// Robotics include files
#include "../utilities/constants.h"
#include "../utilities/structures.h"
#include "../simulator/simulation.h"
#include "../utilities/utilities.h"
#include "../utilities/random.h"
#include "../utilities/inputs.h"
#include "../state_machines/reactive_behavior.h"
#include "../state_machines/state_machine_avoidance.h"
#include "../state_machines/state_machine_destination.h"
#include "../state_machines/state_machine_avoidance_destination.h"
#include "../state_machines/dfs.h"
#include "../state_machines/dijkstra.h"
#include "../state_machines/state_machine_student.h"
#include "../state_machines/campos_potenciales.h"


//Global variables
float K_GOAL=CNT_GOAL*MAG_ADVANCE;
float K_INTENSITY = 1.00*K_GOAL;



// it moves the robot from the origen to the destination using state machines
int go_to(Inputs inputs)
{
 char file_obs[250];
 char file_sensors[250];
 char world_file[250];
 float theta=0;
 int j;
 int i=0,k;
 int ii;
 FILE *fpw;
 FILE *fp_sensors;
 Raw observations;
 Raw inte_quad_obs;
 int flg_inte_quad = 0;
 int flagOnce = 1;
 int flg_finish = 0;
 int tmp;
 float final_x,final_y;
 int num_obs=1;
 float distance1;
 coord coord_robot;
 coord coord_dest;
 int flg;
 int debug=0;
 float xmv,ymv,thetamv;
 int quantized_obs;
 int quantized_attraction;
 int quantized_intensity;
 float intensity = 1.0;
 AdvanceAngle mov_vector;
 int next_state=0;
 int next_state_avoidance=0, next_state_destination=0, next_state_avoidance_destination=0;
 int next_state_fsm_engine=1;
 int state=0;
 AdvanceAngle DistTheta;
 AdvanceAngle mov_vector_avoidance;
 AdvanceAngle mov_vector_destination;
 AdvanceAngle mov_vector_avoidance_destination;
 AdvanceAngle mov_vector_fsm_engine;
 AdvanceAngle mov_vector_potential_fields;
 int selection = 3;
 int pr_out = 0;
 float largest_value;
 int flg_noise=1;
 float noise_advance,noise_angle;
 int label_quantize_outputs=1;
 int flg_result;
 float angle_light;
 int training = TRAINING;
 int flg_vq = 0;
 int size_vq = 4;
 int num_bits_vq = 2;
 char file_hmm[250];
 int flg_omnidirectional = 1;
 //int flg_mdp_angle=0;
 //float angle_mdp;
 int dummy;
 float xc=0.1,yc=0.1;
 float dimx,dimy;
 int num_delay_unk = NUM_DELAY_UNK+1;
 step steps[200];
 int method = 1;  // method variable is used to select the search method, 1 is for first profound search, 2 Dijkstra algorithm 

// it reads the environment file
 sprintf(world_file,"%s%s.wrl",inputs.path,inputs.environment);
 // read_environment in simulator/simulation.h
 read_environment(world_file,debug,&dimx,&dimy);

 // it opens the observation's sensor file to be plot by the graphical interface
 sprintf(file_obs,"%s%s.raw",inputs.path,inputs.output_file);
 #ifdef DEBUG
 printf("\nobservations file: %s\n",file_obs);
 #endif
 if((fpw=fopen(file_obs,"w")) == NULL){
	printf("File %s can not be open\n",file_obs);
       	return(0);
 }	


 fprintf(fpw,"( radio_robot %f )\n",inputs.radio_robot);
#ifdef DEBUG
 printf("\n Radio robot %f \n",inputs.radio_robot);
#endif
 coord_robot.xc=inputs.xo;
 coord_robot.yc=inputs.yo;
 coord_robot.anglec=inputs.angle_robot;
 fprintf(fpw,"( origen %f %f %f )\n",inputs.xo,inputs.yo,inputs.angle_robot);
#ifdef DEBUG
 printf("( origen %f %f %f )\n",inputs.xo,inputs.yo,inputs.angle_robot);
#endif

 
 selection=inputs.selection; 
 //pr_out=inputs.pr_out; 
 largest_value=inputs.largest_value;
 flg_noise=inputs.noise;

 K_GOAL=4.2*CNT_GOAL*inputs.Mag_Advance;
 K_INTENSITY = 1.00*K_GOAL;


 // it moves the robot to the final destination
 flg=0;
 coord_dest.xc=inputs.xd;
 coord_dest.yc=inputs.yd;
 fprintf(fpw,"( destination %f %f )\n",inputs.xd,inputs.yd);
#ifdef DEBUG
 printf("( destination %f %f )\n",inputs.xd,inputs.yd);
#endif

 while(flg == 0){

#ifdef DEBUG
	printf("\n\n ************************************************************\n");
#endif

        distance1=distance(coord_robot,coord_dest);
	// it checks if the robot reached its destination
	if( distance1 < K_GOAL){
#ifdef DEBUG
		printf(" reached distance destination %f\n",distance1);
#endif
                flg= 1;
        }
#ifdef DEBUG
        printf("distance destination %f threshold %f\n",distance1,K_GOAL);
#endif

        // it saves the robot's position
	//fprintf(fpw,"( robot Justina %f %f %f )\n",coord_robot.xc,coord_robot.yc,coord_robot.anglec);
	fprintf(fpw,"( robot student %f %f %f )\n",coord_robot.xc,coord_robot.yc,coord_robot.anglec);
#ifdef DEBUG
	printf("%d Pose robot %f %f %f \n",num_obs,coord_robot.xc,coord_robot.yc,coord_robot.anglec);
#endif



	// ********************************** SENSING  *****************************************************************


	// it gets laser range data from the simulator or the real robot, this function is in ../simulator/simulation.h
        get_sensor_values(coord_robot,inputs.theta_sensor,inputs.range_sensor,&observations,inputs.num_sensors,largest_value);

#ifdef DEBUG
	for(i=0;i<inputs.num_sensors;i++){
       		 printf("range observations.sensors[%d] %f\n",i,observations.sensors[i]);
   	}
#endif
	// it gets the intensity a angle of a light source from the simulator or the real robot, this function is in ../utilities/utilities.h
	get_intensity_angle(coord_robot,coord_dest,&intensity,&angle_light);


        if(flg_noise==1) {
#ifdef DEBUG
 		printf("intensity %f angle light %f \n",intensity,angle_light);
#endif
		// it adds noise to the sensors, function in ../utilities/random.h
		add_noise_obs(&observations,&intensity,&angle_light,inputs.num_sensors,inputs.path);
#ifdef DEBUG
 		printf("with noise intensity %f angle light %f \n",intensity,angle_light);
#endif
	}

#ifdef DEBUG
	for(j=0;j<inputs.num_sensors;j++){
        	printf("noised range sensors[%d] %f\n",j,observations.sensors[j]);
 	}
#endif

	// It quantizes the inputs in file ~/robotics/utilities/utilities.h
	quantized_obs=quantize_inputs(observations,inputs.num_sensors,flg_vq,size_vq,inputs.path);

	// It quantizes the destination in file ~/robotics/utilities/utilities.h
	quantized_attraction=quantize_destination(angle_light,flg_vq);
#ifdef DEBUG
	printf("angle_light %f quantized destination %d \n",angle_light,quantized_attraction);
	printf("quantized inputs %d\n",quantized_obs);
#endif

	// It quantizes the intensity of the destination in file ~/robotics/utilities/utilities.h
 	quantized_intensity = quantize_intensity(intensity,flg_vq);
#ifdef DEBUG
 	printf("intensity %f quantized intensity %d \n",intensity,quantized_intensity);
#endif
	
        // it saves the sensor data to be plot by Python/TK; function in /home/biorobotica/robotics/utilities/utilities.h
        write_obs_sensor(fpw,observations,inputs.sensor,inputs.num_sensors,inputs.theta_sensor,inputs.range_sensor);
	fprintf(fpw,"( sensor destination %d )\n",quantized_attraction);
	fprintf(fpw,"( sensor light %d )\n",quantized_intensity);




	// ********************************** Behaviors *****************************************************************


	if(selection == 1){
                // It calculates the robot's movement using orden cero logic 
                // reactive behavior in ../state_machines/reactive_behavior.h
                //mov_vector_avoidance_destination = reactive_behavior(observations, quantized_attraction, quantized_intensity,inputs.Mag_Advance,inputs.max_angle, inputs.num_sensors);
                DistTheta = reactive_behavior(observations, quantized_attraction, quantized_intensity,inputs.Mag_Advance,inputs.max_angle, inputs.num_sensors);

#ifdef DEBUG
                printf("reactive behavior movement: angle  %f distance %f\n",DistTheta.angle,DistTheta.distance);
#endif
	}

	else if(selection == 2){
		// It calculates the robot's movement using an state machine that only avoids obstacles
		// state_machine_avoidance_destination in ../state_machines/state_machine_avoidance.h
		state=next_state;
		DistTheta = state_machine_avoidance(observations, inputs.num_sensors, state, &next_state,inputs.Mag_Advance,inputs.max_angle);

#ifdef DEBUG
		printf("avoidance behavior movement: angle  %f distance %f\n",DistTheta.angle,DistTheta.distance);
#endif
	}

	else if(selection == 3){
		// It calculates the robot's movement using an state machine that only goes to a light source
		// state_machine_destination in ../state_machines/state_machine_destination.h
		state=next_state;
		DistTheta = state_machine_destination(quantized_attraction, quantized_intensity,state,&next_state,inputs.Mag_Advance,inputs.max_angle);

#ifdef DEBUG
		printf("destination behavior movement: angle  %f distance %f\n",DistTheta.angle,DistTheta.distance);
#endif

	}

	else if(selection == 4){
		// It calculates the robot's movement using an state machine that avoids obstacles and goes to a light source
		state=next_state;
		// state_machine_avoidance_destination in ../state_machines/state_machine_avoidance_destination.h
		DistTheta = state_machine_avoidance_destination(quantized_obs,quantized_attraction,quantized_intensity,state, &next_state,inputs.Mag_Advance,inputs.max_angle);
#ifdef DEBUG
		printf("avoidance destination behavior: angle  %f distance %f\n",DistTheta.angle,DistTheta.distance);
#endif
	}

        else if(selection == 5){
                // It calculates the robot's movement using an state machine created by an student

                // state_machine_avoidance_destination in ../state_machines/state_machine_student.h
                DistTheta = reactive_students(observations, quantized_attraction, quantized_intensity,inputs.Mag_Advance,inputs.max_angle, inputs.num_sensors);

#ifdef DEBUG
                printf("Student reactive behavior movement avoidance destination: angle  %f distance %f\n",DistTheta.angle,DistTheta.distance);
#endif
        }

	else if(selection == 6){
                // It calculates the robot's movement using an state machine created by an student
                state=next_state;
                // state_machine_students in ../state_machines/state_machine_student.h
                DistTheta = state_machine_students(observations, quantized_attraction, quantized_intensity,state,&next_state,inputs.Mag_Advance,inputs.max_angle, inputs.num_sensors,angle_light);

#ifdef DEBUG
                printf("Student FSM behavior avoidance destination: angle  %f distance %f\n",DistTheta.angle,DistTheta.distance);
#endif
        }

	else if(selection == 7){

		    // it finds a path from the origen to a destination using an AI search algorithm
                if(flagOnce)
                {
                    for(i = 0; i < 200; i++) steps[i].node = -1;

		    if(method == 1){

		    		// it finds a path from the origen to a destination using the first search algorithm
                    		sp=dfs(coord_robot.xc,coord_robot.yc,coord_dest.xc,coord_dest.yc,inputs.path,inputs.environment,steps);

		    }
		    else{
		    		// it finds a path from the origen to a destination using the Dijkstra algorithm
                    		sp=dijkstra(coord_robot.xc,coord_robot.yc,coord_dest.xc, coord_dest.yc,inputs.path,inputs.environment,steps);

		    }

		    for(i=0; i < sp-1; i++){
			fprintf(fpw,"( connection %f %f %f %f )\n",steps[i].x, steps[i].y,steps[i+1].x, steps[i+1].y);
			#ifdef DEBUG
        		printf(" Node %d x %f y %f\n", steps[i].node, steps[i].x, steps[i].y);
			#endif
    		    }
		    fprintf(fpw,"( connection %f %f %f %f )\n",steps[i].x, steps[i].y,steps[i].x, steps[i].y);


                    ii = 0;
                    final_x = coord_dest.xc;
                    final_y = coord_dest.yc;
		    fprintf(fpw,"( destination %f %f )\n",steps[ii].x,steps[ii].y);
		    coord_dest.xc= steps[ii].x;
		    coord_dest.yc=steps[ii].y;

		    #ifdef DEBUG
        	    printf("Node %d x %f y %f\n", steps[ii].node, steps[ii].x, steps[ii].y);
                    printf("First light %d: x = %f  y = %f \n",ii,steps[ii].x,steps[ii].y);
		    #endif

                    flagOnce = 0;
                    flg_finish=0;
		    state=next_state;
		    DistTheta.angle=0.0;
		    DistTheta.distance=0.0;
		    
                }
                else
                {
                    if(flg == 1) {
                        if(flg_finish == 1){
				fprintf(fpw,"( distance %f )\n",distance1);
 				fprintf(fpw,"( num_steps %d )\n",num_obs);
 				fclose(fpw);
 				return(num_obs);
		     }
                     else {
                            ii++;
                            if(steps[ii].node != -1)
                            {
				fprintf(fpw,"( destination %f %f )\n",steps[ii].x,steps[ii].y);
                    		coord_dest.xc= steps[ii].x;
                    		coord_dest.yc=steps[ii].y;

                    		#ifdef DEBUG
        	    		printf("Node %d x %f y %f\n", steps[ii].node, steps[ii].x, steps[ii].y);
                    		printf("New Light %d: x = %f  y = %f \n",ii,steps[ii].x,steps[ii].y);
                    		#endif

				flg=0;
                                //printf("type a number \n");
                                //scanf("%d",&tmp);
                            }
                            else
                            {
				fprintf(fpw,"( destination %f %f )\n",final_x,final_y);
                                coord_dest.xc= final_x;
                                coord_dest.yc= final_y;
                                printf("Final Light %d: x = %f  y = %f \n",i,final_x,final_y);
				flg=0;
                                flg_finish = 1;
                            }
                     }
                    }

		    // It calculates the robot's movement using an state machine that avoids obstacles and goes to a light source
                    state=next_state;
                    //DistTheta = state_machine_avoidance_destination(quantized_obs,quantized_attraction,quantized_intensity,state, &next_state,inputs.Mag_Advance,inputs.max_angle);
					DistTheta = campos_potenciales(observations, quantized_attraction, quantized_intensity,inputs.Mag_Advance,inputs.max_angle, inputs.num_sensors,coord_robot.xc,coord_robot.yc,coord_robot.anglec,coord_dest.xc,coord_dest.yc,coord_dest.anglec,distance1);
		    #ifdef DEBUG
               	    printf("avoidance destination behavior: angle  %f distance %f\n",DistTheta.angle,DistTheta.distance);
		    #endif

                }

        }

	else if(selection == 8){

		    DistTheta.angle=0.0;
		    DistTheta.distance=0.0;

        }


        else {
                printf("This behavior does not exist \n");
                exit(0);
        }




	// ****************************************** ACTIONS ***********************************************************************


#ifdef DEBUG
       	printf("movement without noise: angle  %f distance %f\n",DistTheta.angle,DistTheta.distance);
#endif
	if(flg_noise==1){
		// it adds noise to the movement 
        	get_random_advance_angle(&noise_advance,&noise_angle,inputs.path);
		//printf("angle %f\n",DistTheta.angle);
		DistTheta.angle = DistTheta.angle + noise_angle;
		//printf("angle + noise %f\n",DistTheta.angle);
		//printf("distance %f\n",DistTheta.distance);
		DistTheta.distance = DistTheta.distance + noise_advance;
		//printf("distance + noise %f\n",DistTheta.distance);
#ifdef DEBUG
        	printf("movement with noise: angle  %f distance %f\n",DistTheta.angle,DistTheta.distance);
#endif
 	}
        // it saves the robot's actions
        fprintf(fpw,"( movement %f %f )\n",DistTheta.angle,DistTheta.distance);

	// It moves the robot to the desire angle and distance, function in ../utilities/utilities.h"
	flg_result=mvrobot(fpw,DistTheta,&coord_robot);


        num_obs++;
        if(num_obs > inputs.number_steps) flg=1;

 }

 fprintf(fpw,"( distance %f )\n",distance1);
 fprintf(fpw,"( num_steps %d )\n",num_obs);
 fclose(fpw);

 return(num_obs);
}




// Main program
int main(int argc, char *argv[])
{

 Inputs inputs;
 int num_steps;


 // it gets line inputs, function in ../utilities/inputs.h 
 get_inputs(argc,argv,&inputs);

 //it sends the robot to the asked position
 num_steps=go_to(inputs);

 return 0;

}



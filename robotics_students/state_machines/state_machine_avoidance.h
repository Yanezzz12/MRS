/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_avoidance.h			*
 *                                                      *
 *		Jesus Savage				*
 *		FI-UNAM					*
 *		6-1-2015                                *
 *                                                      *
 ********************************************************/



// Function to get next state
AdvanceAngle state_machine_avoidance(Raw observations, int num_sensors, int state, int *next_state,float Mag_Advance,float max_angle){

 AdvanceAngle gen_vector;
 int obs;
 int j;
 float left_side=0;
 float right_side=0;
 int value = 0;


 for(j=0;j<num_sensors/2;j++){
	right_side = observations.sensors[j] + right_side; 
#ifdef DEBUG
	printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
#endif
 }

 for(j=num_sensors/2;j<num_sensors;j++){
	left_side = observations.sensors[j] + left_side; 
#ifdef DEBUG
	printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
#endif
 }

 right_side = right_side/(num_sensors/2);
 left_side = left_side/(num_sensors/2);
#ifdef DEBUG
 printf("Average right side %f\n",right_side);
 printf("Average left side %f\n",left_side);
#endif

 if( left_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 if( right_side < THRS) value = (value << 1) + 1;
 else value = (value << 1) + 0;

 obs = value;
#ifdef DEBUG
 printf("obs %d\n",obs);
#endif


 switch ( state ) {

        case 0:
                if (obs == 0){
			// there is not obstacle
                        gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        printf("Present State: %d FORWARD\n", state);
                        *next_state = 0;
                }
                else{
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        printf("Present State: %d STOP\n", state);

                        if (obs == 1){
                                // obtacle in the right
                                *next_state = 1;
                        }
                        else if (obs == 2){
                                // obstacle in the left
                                *next_state = 3;
                        }
                        else if (obs == 3){
                                // obstacle in the front
                                *next_state = 5;
                        }
                }

                break;

        case 1: // Backward, obstacle in the right
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                printf("Present State: %d BACKWARD, obstacle RIGHT\n", state);
                *next_state = 2;
                break;

        case 2: // left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                printf("Present State: %d TURN LEFT\n", state);
                *next_state = 0;
                break;

        case 3: // Backward, obstacle in the left
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                printf("Present State: %d BACKWARD, obstacle LEFT\n", state);
                *next_state = 4;
                break;

        case 4: // right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                printf("Present State: %d TURN RIGHT\n", state);
                *next_state = 0;
                break;
        case 5: // Backward, obstacle in front
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
                printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
                *next_state = 6;
                break;

        case 6: /// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                printf("Present State: %d TURN 1 LEFT\n", state);
                *next_state = 7;
                break;

        case 7:// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                printf("Present State: %d TURN 2 LEFT\n", state);
                *next_state = 8;
                break;


        case 8: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                printf("Present State: %d 1 FORWARD\n", state);
                *next_state = 9;
                break;

        case 9: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                printf("Present State: %d 2 FORWARD\n", state);
                *next_state = 10;
                break;

        case 10: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                printf("Present State: %d TURN 1 RIGHT\n", state);
                *next_state = 11;
                break;

        case 11: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                printf("Present State: %d TURN 2 RIGHT\n", state);
                *next_state = 0;
                break;
 }

 //printf("Next State: %d\n", *next_state);
 return gen_vector;

}



                 


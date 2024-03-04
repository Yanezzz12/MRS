/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_avoidance_destination.h          	*
 *                                                      *
 *		Jesus Savage				*
 *		FI-UNAM					*
 *		5-2-2015                                *
 *                                                      *
 ********************************************************/


// State Machine 
AdvanceAngle state_machine_avoidance_destination(int obs, int dest, int intensity, int state, int *next_state,float Mag_Advance,float max_angle){

 AdvanceAngle gen_vector;

#ifdef DEBUG
 printf("obs intensity %d obstacles %d dest %d\n",intensity,obs,dest);
#endif

 switch ( state ) {

        case 0:
                if (intensity == 1){
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
			//gen_vector=generate_output(FORWARD/4,Mag_Advance,max_angle);
#ifdef DEBUG
                        printf("Present State: %d STOP\n", state);
			printf("\n **************** Reached light source ******************************\n");
#endif
                        *next_state = 1;
                }
                else{

			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
#ifdef DEBUG
                        printf("Present State: %d FORWARD\n", state);
#endif
                        *next_state = 1;
                }

                break;

        case 1:
                if (obs == 0){
			// there is not obstacle
                        gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
#ifdef DEBUG
                        printf("Present State: %d FORWARD\n", state);
#endif
                        *next_state = 13;
                }
                else{
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
#ifdef DEBUG
                        printf("Present State: %d STOP\n", state);
#endif

                        if (obs == 1){
                                // obtacle in the  right
                                *next_state = 2;
                        }
                        else if (obs == 2){
                                // obtacle in the left
                                *next_state = 4;
                        }
                        else if (obs == 3){
				// obstacle in the front
                                *next_state = 6;
                        }
                }

                break;

        case 2: // Backward, obstacle in the right
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
#ifdef DEBUG
		printf("Present State: %d BACKWARD, obstacle right\n", state);
#endif
                *next_state = 3;
                break;

        case 3: // right turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
#ifdef DEBUG
		printf("Present State: %d TURN LEFT\n", state);
#endif
                *next_state = 0;
                break;

        case 4: // Backward, obstacle in the left
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
#ifdef DEBUG
		printf("Present State: %d BACKWARD, obstacle left\n", state);
#endif
                *next_state = 5;
                break;

        case 5: // left turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
#ifdef DEBUG
		printf("Present State: %d TURN RIGTH\n", state);
#endif
                *next_state = 0;
                break;

        case 6: // Backward, obstacle in front
                gen_vector=generate_output(BACKWARD,Mag_Advance,max_angle);
#ifdef DEBUG
		printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
#endif
                *next_state = 7;
                break;

	case 7: /// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
#ifdef DEBUG
		printf("Present State: %d TURN 1 LEFT\n", state);
#endif
                *next_state = 8;
                break;

        case 8:// Left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
#ifdef DEBUG
		printf("Present State: %d TURN 2 LEFT\n", state);
#endif
                *next_state = 9;
                break;

        case 9: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
#ifdef DEBUG
                printf("Present State: %d 1 FORWARD\n", state);
#endif
                *next_state = 10;
                break;

        case 10: // Forward
                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
#ifdef DEBUG
                printf("Present State: %d 2 FORWARD\n", state);
#endif
                *next_state = 11;
                break;

	case 11: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
#ifdef DEBUG
                printf("Present State: %d turn 1 RIGHT\n", state);
#endif
                *next_state = 12;
                break;

        case 12: // Right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
#ifdef DEBUG
                printf("Present State: %d turn 2 RIGHT\n", state);
#endif
                *next_state = 0;
                break;


        case 13: // // check destination
		 if (dest == 0){
                                // go right
                                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
#ifdef DEBUG
                                printf("Present State: %d RIGHT\n", state);
#endif
                                *next_state = 5;
                 }
                 else if (dest == 1){
                                // go left
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
#ifdef DEBUG
                                printf("Present State: %d LEFT\n", state);
#endif
                                *next_state = 3;
                 }
                 else if (dest == 2){
                                // go right single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
#ifdef DEBUG
                                printf("Present State: %d FORWARD\n", state);
#endif
                                *next_state = 5;
                 }
                 else if (dest == 3){
                                // go left single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
#ifdef DEBUG
                                printf("Present State: %d FORWARD\n", state);
#endif
                                *next_state = 3;
                 }
                break;

	default:
		printf("State %d not defined used ", state);
                gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                next_state = 0;
                break;

                
 }

 //printf("Next State: %d\n", *next_state);
 return gen_vector;

}



                 


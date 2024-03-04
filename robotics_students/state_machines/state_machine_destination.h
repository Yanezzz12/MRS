/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_destination.h                    	*
 *                                                      *
 *		Jesus Savage				*
 *		FI-UNAM					*
 *		6-1-2015                                *
 *                                                      *
 ********************************************************/


// Function to get next state
AdvanceAngle state_machine_destination(int dest, int intensity, int state, int *next_state,float Mag_Advance,float max_angle){

 AdvanceAngle gen_vector;

 printf("obs intensity %d dest %d\n",intensity,dest);

 switch ( state ) {

        case 0:
                if (intensity == 1){
                        gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                        printf("Present State: %d STOP\n", state);
                        *next_state = 0;
                }
                else{

			gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                        printf("Present State: %d FORWARD\n", state);
                        *next_state = 1;
                }

                break;

        case 1: // It checks for the destination
                 if (dest == 0){
                                // go right
                                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                                printf("Present State: %d RIGHT\n", state);
                                *next_state = 2;
                 }
                 else if (dest == 1){
                                // go left
                                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
                                printf("Present State: %d LEFT\n", state);
                                *next_state = 3;
                 }
                 else if (dest == 2){
                                // go right single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                printf("Present State: %d RIGHT\n", state);
                                *next_state = 2;
                 }
                 else if (dest == 3){
                                // go left single
                                gen_vector=generate_output(FORWARD,Mag_Advance,max_angle);
                                printf("Present State: %d LEFT\n", state);
                                *next_state = 3;
                 }

                break;


        case 2: // right turn
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
		printf("Present State: %d TURN RIGHT\n", state);
                *next_state = 0;
                break;

        case 3: // left turn
                gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
		printf("Present State: %d TURN LEFT\n", state);
                *next_state = 0;
                break;
                
	default:
                printf("State %d not defined used ", state);
                gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                *next_state = 0;
                break;

 }

 //printf("Next State: %d\n", *next_state);
 return gen_vector;

}


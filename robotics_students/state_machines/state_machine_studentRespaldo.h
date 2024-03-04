/********************************************************
 *                                                      *
 *                                                      *
 *      state_machine_student.h          				*
 *                                                      *
 *		Student:										*
 *		FI-UNAM											*
 *		2-15-2024                               		*
 *                                                      *
 ********************************************************/

/*Notes:
>> Robot has two sides left & right
>> This is the behaviour 5
>> How does it work?
	Save all changes in state_machine_student.h
	With "./robotics_students_make" compile program in /user/robotics_students
	With command "make" compile program in "/user/robotics_students/motion_planner"
	Go to "user/robotics_students/gui" and execute "python2.7 GUI_robotics_students.py 5"
	If everything is correct, the programed behaviour will work effectively
*/

/*Variables to change
>>
>>
>>
>>
>>
*/

// Student State Machine 
AdvanceAngle reactive_students(Raw observations, int dest, int intensity, float Mag_Advance, float max_angle, int num_sensors)
{
	AdvanceAngle gen_vector;
 	int obs;
 	int j;
	float left_side = 0;
 	float right_side = 0;
 	int value = 0;
 	static int step = 0;

 	step++;
 	printf("\n\n **************** Student Reactive Behavior %d *********************\n", step);

 	//Left and right sensing
 	for(j = 0; j < num_sensors/2;j++)
 	{
        right_side = observations.sensors[j] + right_side;
        printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
 	}
 	for(j = num_sensors/2;j < num_sensors;j++)
 	{
    	left_side = observations.sensors[j] + left_side;
        printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
 	}

 	right_side = right_side/(num_sensors/2);
 	left_side = left_side/(num_sensors/2);
 	printf("Average right side %f\n",right_side);
 	printf("Average left side %f\n",left_side);

 	if( left_side < THRS) value = (value << 1) + 1;
 	else value = (value << 1) + 0;

 	if( right_side < THRS) value = (value << 1) + 1;
 	else value = (value << 1) + 0;

 	obs = value;
 	printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);

	
 	/* if(intensity == 1) //===========LIGHT FOLLOWER ALGORITHM=============
 	{	
		//==========STOPS MOVEMENT WHEN LIGHT HAS BEEN REACHED============
		// Constants STOP, TURN RIGHT, ETC, are defined in ../utilities/constants.h
		// generate_output function in ../utilities/utilities.h
		gen_vector = generate_output(STOP,Mag_Advance,max_angle);
        printf("STOP\n");
		printf("\n **************** Reached light source ******************************\n");
		// 
	} */
 	if(obs == 0) //obs = 0: There's no obstacle 
 	{
        //----ADDED----//
		gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
		printf("FORWARD\n");
		//----END OF ADDED----//

		/*
		if(dest == 0) //Obstacle in the left
		{
	        //Go right twice
			gen_vector = generate_output(RIGHTADVANCETWICE,Mag_Advance,max_angle);
	        printf("TURN RIGHT TWICE\n");
    	}
	    else if(dest == 1) //Obstacle in the right
	    {
	        //Go left twice
	        gen_vector = generate_output(LEFTADVANCETWICE,Mag_Advance,max_angle);
	        printf("TURN LEFT TWICE\n");
	    }
	    else if (dest == 2) //Obstacle in the left
	    {
	        // go right
	        gen_vector = generate_output(RIGHTADVANCE,Mag_Advance,max_angle);
	        printf("TURN RIGHT\n");
	    }
	    else if (dest == 3) //Obstacle in the right
	    {
	        // go left
	        gen_vector=generate_output(LEFTADVANCE,Mag_Advance,max_angle);
	        printf("TURN LEFT\n");
	    } */
 	} //===========END OF LIGHT FOLLOWER ALGORITHM=============// 
	else if (obs == 1)
 	{
        // Obtacle in the right
        gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
		printf("TURN LEFT\n");
 	}
 	else if (obs == 2)
 	{
        //Obstacle in the left
		gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
		printf("TURN RIGHT\n");
 	}
 	else if (obs == 3)
 	{
		// obstacle in the front
        gen_vector=generate_output(LEFT,Mag_Advance,max_angle);
		printf("TURN LEFT\n");
 	}
	return gen_vector;
}

// Student State Machine 
AdvanceAngle state_machine_students(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors){

 	AdvanceAngle gen_vector;
 	int obs;
 	int j;
 	float left_side=0;
 	float right_side=0;
 	int value = 0;

 	printf("\n\n **************** Student State Machine *********************\n");

 	for(j = 0;j < num_sensors/2;j++)
 	{
        right_side = observations.sensors[j] + right_side;
        printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
 	}

 	for(j = num_sensors/2;j < num_sensors;j++)
 	{
        left_side = observations.sensors[j] + left_side;
        printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
 	}

	right_side = right_side/(num_sensors/2);
	left_side = left_side/(num_sensors/2);
	printf("Average right side %f\n",right_side);
	printf("Average left side %f\n",left_side);

 	if( left_side < THRS) value = (value << 1) + 1;
 	else value = (value << 1) + 0;

 	if( right_side < THRS) value = (value << 1) + 1;
 	else value = (value << 1) + 0;

 	obs = value;
 	printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);

 	switch ( state ) 
 	{
        case 0:
        	/*if (intensity == 1)
            {
                gen_vector = generate_output(STOP,Mag_Advance,max_angle);
                *next_state = 1;
                printf("Present State: %d STOP\n", state);
				printf("\n **************** Reached light source ******************************\n");
            }
            else 
            {
				gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 1;
                printf("Present State: %d FORWARD\n", state);
            }*/

			//----ADDED----//
			gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
            *next_state = 1;
            printf("Present State: %d FORWARD\n", state);
            //----END OF ADDED----//
			break;
        case 1:
            if (obs == 0)
            {
				// There's not obstacle
                gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 13;
                printf("Present State: %d FORWARD\n", state);
            }
            else
            {
                gen_vector=generate_output(STOP,Mag_Advance,max_angle);
                printf("Present State: %d STOP\n", state);

                if (obs == 1)
                {
                	//Obstacle in the right
                    *next_state = 2;
                }
                else if (obs == 2)
                {
                	//Obstacle in the left
                    *next_state = 4;
                }
                else if (obs == 3)
                {
					//Obstacle in the front
                    *next_state = 6;
                }
            }
            break;
        case 2: //Backward, obstacle in the right
            gen_vector = generate_output(BACKWARD,Mag_Advance,max_angle);
            *next_state = 3;
			printf("Present State: %d BACKWARD, obstacle right\n", state);
            break;
        case 3: //Right turn
            gen_vector = generate_output(LEFT,Mag_Advance,max_angle);
            *next_state = 0;
			printf("Present State: %d TURN LEFT\n", state);
            break;
        case 4: //Backward, obstacle in the left
            gen_vector = generate_output(BACKWARD,Mag_Advance,max_angle);
            *next_state = 5;
			printf("Present State: %d BACKWARD, obstacle left\n", state);
            break;
        case 5: // left turn
            gen_vector = generate_output(RIGHT,Mag_Advance,max_angle);
            *next_state = 0;
			printf("Present State: %d TURN RIGTH\n", state);
            break;
        case 6: // Backward, obstacle in front
            gen_vector = generate_output(BACKWARD,Mag_Advance,max_angle);
            *next_state = 7;
			printf("Present State: %d BACKWARD, obstacle FRONT\n", state);
            break;
		case 7: // Left turn
            gen_vector = generate_output(LEFT,Mag_Advance,max_angle);
            *next_state = 8;
			printf("Present State: %d TURN 1 LEFT\n", state);
            break;
        case 8:// Left turn
            gen_vector = generate_output(LEFT,Mag_Advance,max_angle);
            *next_state = 9;
			printf("Present State: %d TURN 2 LEFT\n", state);
            break;
        case 9: // Forward
            gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
            *next_state = 10;
            printf("Present State: %d 1 FORWARD\n", state);
            break;
        case 10: // Forward
            gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
            *next_state = 11;
            printf("Present State: %d 2 FORWARD\n", state);
            break;
		case 11: // Right turn
            gen_vector = generate_output(RIGHT,Mag_Advance,max_angle);
            *next_state = 12;
            printf("Present State: %d turn 1 RIGHT\n", state);
            break;
        case 12: // Right turn
            gen_vector = generate_output(RIGHT,Mag_Advance,max_angle);
            *next_state = 0;
            printf("Present State: %d turn 2 RIGHT\n", state);
            break;
        case 13: // // check destination
		 	if (dest == 0)
		 	{
            	// go right
                gen_vector=generate_output(RIGHT,Mag_Advance,max_angle);
                *next_state = 5;
				printf("Present State: %d RIGHT\n", state);
            }
            else if (dest == 1)
            {
            	// go left
                gen_vector = generate_output(LEFT,Mag_Advance,max_angle);
                *next_state = 3;
				printf("Present State: %d LEFT\n", state);
            }
            else if (dest == 2)
            {
            	// go right single
                gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 5;
                printf("Present State: %d FORWARD\n", state);
            }
           	else if (dest == 3)
           	{
            	// go left single
            	gen_vector = generate_output(FORWARD,Mag_Advance,max_angle);
                *next_state = 3;
                printf("Present State: %d FORWARD\n", state);
            }
            break;
	default:
		printf("State %d not defined used ", state);
        gen_vector = generate_output(STOP,Mag_Advance,max_angle);
        next_state = 0;
        break;  
	}
	return gen_vector;
}



                 


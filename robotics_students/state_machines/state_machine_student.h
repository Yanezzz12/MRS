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

/*----ADDED----//
	Observations -> ??
	intensity -> 0: far frome light, 1: close to light
	obs -> 0: There's no obstacle, 1: Obstacle in the right, 2: Obstacle in the left, 3: Obstacle in front
	dest ->
	Mag_Advance -> ???
	max_angle -> ???

	Movement direction is given by
	gen_vector = generate_output(MOVEMENT,Mag_Advance,max_angle);
	MOVEMENT: FORWARD/RIGHTADVANCETWICE/LEFTADVANCETWICE/RIGHTADVANCE/LEFTADVANCE
//----END OF ADDED----*/



float slopeEquation(float m, float b, float x)
{
	float y = m * x + b;
	return y;
}

//Student State Machine 1 (python2.7 GUI_robotics_students.py 5)
float m, angle, slope, intercept;
int obstacleEncountered = 0;
coord obs_coord = {0.0f, 0.0f, 0.0f}; //Default conditions
coord initial_position = {0.0f, 0.0f, 0.0f}; 
AdvanceAngle reactive_students(Raw observations, int dest, int intensity, float Mag_Advance, float max_angle, int num_sensors, float angle_light, coord coord_robot, coord coord_dest)
{
	AdvanceAngle gen_vector;
 	int obs;
 	int j;
	float left_side = 0;
 	float right_side = 0;
 	int value = 0;
 	static int step = 0;

	//----ADDED CODE----//
	if(step == 0)
	{
		initial_position = coord_robot;
		slope = (coord_dest.yc - initial_position.yc)/(coord_dest.xc - initial_position.xc); 	//Slope
		intercept = initial_position.yc - slope * initial_position.xc; 							//Intercept
	}
	//----END OF ADDED CODE----//

 	printf("\n\n **************** Student Reactive Behavior %d *********************\n", step);

 	//Left & right sensing
 	for(j = 0; j < num_sensors/2;j++)
 	{
        right_side = observations.sensors[j] + right_side;
        printf("Right side sensor[%d] %f\n",j,observations.sensors[j]);
 	}
 	for(j = num_sensors/2;j < num_sensors;j++)
 	{
    	left_side = observations.sensors[j] + left_side;
        printf("Left side sensor[%d] %f\n",j,observations.sensors[j]);
 	}

 	right_side = right_side/(num_sensors/2);
 	left_side = left_side/(num_sensors/2);
	printf("Average right side %f\n",right_side);
 	printf("Average left side %f\n",left_side);

 	if(left_side < THRS) value = (value << 1) + 1;
 	else value = (value << 1) + 0;

 	if(right_side < THRS) value = (value << 1) + 1;
 	else value = (value << 1) + 0;

 	obs = value;
 	printf("intensity %d obstacles %d dest %d\n",intensity,obs,dest);

	//----ADDED CODE----//
	/*Notes:
		>> I need coordinates
		>> If coords hasn't been declared, robot follows angle
		>> When obstacle found, register coordinates and turn left to round object
		>> When coordinates are equal to previously saved, follow angle light direction again
		>> Repeat past step if necessary
	*/

	if(obs == 0) //No obstacle
	{
		if(obstacleEncountered != 0)
		{
			if(right_side < 0.05)
			{
				gen_vector = generate_output(FORWARD, Mag_Advance, max_angle);
			}
			else if(right_side >= 0.05)
			{
				gen_vector = MoveRobot(Mag_Advance, -20);
			}
			if(std::trunc(coord_robot.yc)/100 == std::trunc(slopeEquation(slope, intercept, coord_robot.xc)))
			{
				gen_vector = MoveRobot(Mag_Advance, angle_light);
				obstacleEncountered = 0;
			}
		}
		else 
		{
			gen_vector = generate_output(FORWARD, Mag_Advance, max_angle);
		}
	}
	else if(obs == 1) //Obstacle in the right 
	{
		gen_vector = generate_output(LEFTADVANCE, Mag_Advance, max_angle);
	}
	else if(obs == 2) //Obstacle in the left
	{
		gen_vector = generate_output(RIGHTADVANCE, Mag_Advance, max_angle);
	}
	else if(obs == 3) //Obstacle in front
	{
		obstacleEncountered = 1;
		gen_vector = generate_output(LEFTADVANCETWICE, Mag_Advance, max_angle);
	}

	if(step == 0)
		gen_vector = MoveRobot(Mag_Advance, angle_light);

	//----END OF ADDED CODE----//

	step++;
	return gen_vector;
}

//Student State Machine 2 (python2.7 GUI_robotics_students.py 6)
coord fq = {0.00001, 0.00001f, 0.0f};
coord previousPosition = {0.001f, 0.001f, 0.0f};
AdvanceAngle state_machine_students(Raw observations, int dest, int intensity, int state, int *next_state, float Mag_Advance, float max_angle, int num_sensors, coord coord_robot, coord coord_dest)
{
 	AdvanceAngle gen_vector;
 	int obs;
 	int j;
	float left_side=0;
 	float right_side=0;
 	int value = 0;

	//Added variables
	float E1 = 0.5f;
	float delta = 1.0f;
	float Uatr;
	coord Fatr;
	coord newPosition;
	float angleDirection;
	//End of added variables

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

	//----ADDED CODE----//
	Uatr = (0.5) * E1 * (pow(coord_robot.xc - coord_dest.xc,2) + pow(coord_robot.yc - coord_dest.yc, 2));
	Fatr = {E1 * (coord_robot.xc - coord_dest.xc), E1 * (coord_robot.yc - coord_dest.yc), 0.0f};

	newPosition.xc = previousPosition.xc - delta * fq.xc;
	newPosition.yc = previousPosition.yc - delta * fq.yc;

	previousPosition = coord_robot;
	fq = unitaryVector(Fatr); 

	angleDirection = angleVector(newPosition);

	gen_vector = MoveRobot(0.02, angleDirection);

	//Coordenadas: coord_robot, coord coord_dest
	//----END OF ADDED CODE----//
 	
	return gen_vector;
}



                 


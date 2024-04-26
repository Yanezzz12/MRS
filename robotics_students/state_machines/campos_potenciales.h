/********************************************************
 *                                                      *
 *                                                      *
 *      campos potenciales.h          			*
 *                                                      *
 *              Rubio Ortiz Rodrigo Yael                *
 *		fI-UNAM					*
 *		2-15-2024                               *
 *                                                      *
 ********************************************************/



 
// campos_potenciales
AdvanceAngle campos_potenciales(Raw observations, int dest, int intensity, float Mag_Advance, float max_angle, int num_sensors,float Robot_x, float Robot_y, float Robot_angle,float Dest_x, float Dest_y, float Dest_angle,float distance){

       AdvanceAngle gen_vector;
       int obs;
       int j;
       float left_side=0;
       float right_side=0;
       float back_side=0;
       float front_side=0;
       int obs_back = 0;
       int obs_front = 0;
       int obs_left = 0;
       int obs_right = 0; 
       int value = 0;

       // campos potenciales
       //atractive
       float q_atractive_x, q_atractive_y, q_atractive_module;
       float F_atractive_x, F_atractive_y;

       float F_atractive_robot_x, F_atractive_robot_y;
       float rotacion_robot;
       
       float angle_q;
       float di = 0.1 ; //di 0.20 UN CUARTO DEL MAPA 1 
       float e1 = 0.17 ; //ei 0.065 funciona pero lento 0.15 es aceptable sin fuerza repulsiva

       //repulsive
       float angle_rob_obs;
       float origin_angle = 2.35;
       float range_sensor = 4.7122;
       float q_obs_x[num_sensors], q_obs_y[num_sensors], q_obs_module[num_sensors];
       float angle_obstacle[num_sensors];
       float F_repulsive_x[num_sensors], F_repulsive_y[num_sensors];
       float F_repulsive_acumulado_x=0,F_repulsive_acumulado_y=0;

       float F_repulsive_module = 0, F_repulsive_angle = 0, F_repulsive_x_origen = 0, F_repulsive_y_origen = 0;
       float n = 0.00020; //0.015 funciona bien
       float d0 = 0.07; //0.05 Es la distancia maxima para no actuar
       bool humbral[num_sensors] = { false };

       //SUMA de fuerzas
       float f_total_x, f_total_y;
       float f_module, f_angle, f_advance;

       float delta = 0.1 ;

       //float PI = 3.14159;



       printf("\n\n **************** CAMPOS POTENCIALES *********************\n");


       //Variables control CAMPOS POTENCIAS
       float D = distance;

       printf("Robot_x %f\n",Robot_x);
       printf("Robot_y %f\n",Robot_y);
       printf("Robot_angle %f\n",Robot_angle);

       printf("Dest_x %f\n",Dest_x);
       printf("Dest_y %f\n",Dest_y);
       printf("Dest_angle %f\n",Dest_angle);

       printf("Distance %f\n",back_side);

       for(j=0;j<num_sensors/2;j++){
              right_side = observations.sensors[j] + right_side;
              printf("right side sensor[%d] %f\n",j,observations.sensors[j]);
       }

       for(j=num_sensors/2;j<num_sensors;j++){
              left_side = observations.sensors[j] + left_side;
              printf("left side sensor[%d] %f\n",j,observations.sensors[j]);
       }

       for(j=num_sensors;j<num_sensors/4;j++){
              back_side = observations.sensors[j] + back_side;
              printf("back side sensor[%d] %f\n",j,observations.sensors[j]);
       }

       for(j=num_sensors*(3/4);j<num_sensors/4;j++){
              back_side = observations.sensors[j] + back_side;
              printf("back side sensor[%d] %f\n",j,observations.sensors[j]);
       }



       right_side = right_side/(num_sensors/2);
       left_side = left_side/(num_sensors/2);
       front_side = front_side/(num_sensors/2);
       back_side = back_side/(num_sensors/2);
       printf("Average right side %f\n",right_side);
       printf("Average left side %f\n",left_side);
       printf("Average front side %f\n",front_side);
       printf("Average back side %f\n",back_side);

       if( left_side < THRS) value = (value << 1) + 1;
       else value = (value << 1) + 0;

       if( right_side < THRS) value = (value << 1) + 1;
       else value = (value << 1) + 0;


       if( back_side < THRS){
              obs_back = 1;
       }

       if( front_side < THRS){
              obs_front = 1;
       }
       if( right_side < THRS){
              obs_left = 1;
       }
       
       obs = value;


       printf("obs = %f\n",obs);


       // atractive FEED

       q_atractive_x        = Dest_x-Robot_x;
       q_atractive_y        = Dest_y-Robot_y;
       angle_q              = atan2(q_atractive_y, q_atractive_x);
       q_atractive_module   = sqrt(q_atractive_x * q_atractive_x+ q_atractive_y * q_atractive_y);
       
       if(q_atractive_module > di){
       F_atractive_x = e1*q_atractive_x/q_atractive_module;
       F_atractive_y = e1*q_atractive_y/q_atractive_module;
       }else{
       F_atractive_x = e1*q_atractive_x;
       F_atractive_y = e1*q_atractive_y;
       }

       q_atractive_module   = sqrt(F_atractive_x * F_atractive_x+ F_atractive_y * F_atractive_y);
       rotacion_robot = angle_q -Robot_angle; 
       printf("el vector q atractivo es: %f, %f, con angulo: %f, angulo robot: %f y debe girar: %f \n", q_atractive_x, q_atractive_y, angle_q, Robot_angle, rotacion_robot);


       //se realiza una transformaciṕn lineal para poner el vector de atracción en la perpectiva del
       //marco de referencia del robot 

       F_atractive_robot_x = q_atractive_module*cos(rotacion_robot);
       F_atractive_robot_y = q_atractive_module*sin(rotacion_robot);





       //F_atractive_x = 0;
       //F_atractive_y = 0;


       //repulsive FEED

       for(int i=0;i< num_sensors;i++){
              //angle_rob_obs = 3.14159-Robot_angle-(3.14159/4)-(3.14159*3/(4*16))*i;

              angle_rob_obs = PI+(PI/4)+(PI*3/(2*(num_sensors-1)))*i; 
              
              //if(observations.sensors[j]<0.08){
              q_obs_x[i] =  observations.sensors[j]*cos(angle_rob_obs);
              q_obs_y[i] =  observations.sensors[j]*sin(angle_rob_obs);

              if (observations.sensors[i]<=d0){
                     q_obs_module[i]   = observations.sensors[j];     
                     F_repulsive_x[i] = n*(q_obs_x[i])*(1/(pow(q_obs_module[i], 3)))*((1/q_obs_module[i])-(1/d0));
                     F_repulsive_y[i] = n*(q_obs_y[i])*(1/(pow(q_obs_module[i], 3)))*((1/q_obs_module[i]) - (1/d0));  
                     humbral[i]={true};

                     if (i<(num_sensors/2)){
                            F_repulsive_x[i] = -F_repulsive_x[i];
                            F_repulsive_y[i] = -F_repulsive_y[i];
                     }

              }
              else{
                     q_obs_module[i] =  d0;
                     F_repulsive_x[i] = 0;
                     F_repulsive_y[i] = 0;
                     humbral[i]={false};
              }

              F_repulsive_acumulado_x = F_repulsive_acumulado_x + F_repulsive_x[i];
              F_repulsive_acumulado_y = F_repulsive_acumulado_y + F_repulsive_y[i];

              angle_obstacle[i]     = (180*atan2(q_obs_y[i], q_obs_x[i]))/3.14159;
              
              F_repulsive_module  = sqrt((F_repulsive_x[i] * F_repulsive_x[i]) + (F_repulsive_y[i] * F_repulsive_y[i]));
              F_repulsive_angle   = atan2(F_repulsive_y[i], F_repulsive_x[i]);
              F_repulsive_angle   = (F_repulsive_angle/PI)*180;


              printf("Sensor %i x %0.2f ,y %0.2f, ang_grad %0.2f, F_rep x %0.2f,y %0.2f, mod %0.2f, F_ang %0.2f, humbral %i \n",i,q_obs_x[i],q_obs_y[i],angle_obstacle[i] ,F_repulsive_x[i],F_repulsive_y[i],F_repulsive_module,F_repulsive_angle,humbral[i]? true :false);
       }
              /*
              //se aplica una transformación lineal de rotacion 
              F_repulsive_x_origen = F_repulsive_module*cos(F_repulsive_angle-Robot_angle);
              F_repulsive_y_origen = F_repulsive_module*sin(F_repulsive_angle-Robot_angle);
              */
       
              F_repulsive_module  = sqrt((F_repulsive_acumulado_x * F_repulsive_acumulado_x) + (F_repulsive_acumulado_y*F_repulsive_acumulado_y));
              F_repulsive_angle   = atan2(F_repulsive_acumulado_y, F_repulsive_acumulado_x);
              F_repulsive_angle   = (F_repulsive_angle/PI)*180;
       printf("El vector repulsivo total es: x %0.2f, y %0.2f, module %0.2f con angulo %0.2f \n", F_repulsive_acumulado_x,F_repulsive_acumulado_y, F_repulsive_module,F_repulsive_angle);


       //SUM OF FORCES

/*
        f_total_x = F_atractive_x + F_repulsive_acumulado_x;
        f_total_y = F_atractive_y + F_repulsive_acumulado_y;
*/


// conciderando la rotacion hacia el marco de referencia del origen 
/*       f_total_x = F_atractive_x +  F_repulsive_x_origen ;
       f_total_y = F_atractive_y +  F_repulsive_y_origen ;
*/


       //Considerando suma en marco de referencia del robot
       f_total_x = F_atractive_robot_x + F_repulsive_acumulado_x;
       f_total_y = F_atractive_robot_y + F_repulsive_acumulado_y;


       f_module  = sqrt(f_total_x * f_total_x+ f_total_y * f_total_y);
       f_angle   = atan2(f_total_y, f_total_x);
       f_advance = delta*f_module ;



//       rotacion_robot = f_angle -Robot_angle; //marco de referencia origen


       rotacion_robot = f_angle ; //marco de referencia robot 

       printf("f_total_x: %.2f, f_total_y: %.2f, f_module: %.2f, f_angle: %.2f,Robot_angle: %f, rotacion_robot: %f, f_advance: %.2f\n", f_total_x, f_total_y, f_module, f_angle, Robot_angle,rotacion_robot, f_advance);

       //funcion para saturar el actuador
       if(f_advance>0.1){
              f_advance = 0.1;
       }

       if ((rotacion_robot >=0)||((rotacion_robot <=PI))){
	gen_vector=generate_output(LEFTADVANCE,f_advance,rotacion_robot );
       }
       else{
	gen_vector=generate_output(RIGHTADVANCE,f_advance,rotacion_robot );
       }

//bloqueo total
	//gen_vector=generate_output(RIGHTADVANCE,0,0 );

 return gen_vector;

}



                 


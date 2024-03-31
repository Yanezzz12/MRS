
/********************************************************************************
*	utilities.h                                                           	*
* 									     	*
*                               J. Savage                                    	*
*                                                                        	*
*                               FI-UNAM 2015                                 	*
********************************************************************************/

#define NUM_MAX_SENSORS 512
#define NUM_MAX_CENTROIDS 1024
//#define TYPE_0 2
#define TYPE_0 0


extern float K_INTENSITY;


// it writes the sensors' readings to be plot by Tk
int write_obs_sensor(FILE *fpw,Raw observations,char *sensor,int num_sensors, float start_angle,float range){
 int j;


 fprintf(fpw,"( sensor %s %d %f %f",sensor,num_sensors,range,start_angle); 

 for(j=0;j<num_sensors;j++){
        fprintf(fpw," %f",observations.sensors[j]);
        //printf("laser[%d] %f\n",j,observations.sensors[j]);
 }

 fprintf(fpw," )\n");
 //printf("\n");

 return(1);
}


// it saves the sensor data to be used by the VQ 
 int write_sensor_data(FILE *fp_sensors, float intensity, float angle_light, int num_sensors, Raw observations, int flg_inte_quad){

 int j;


 fprintf(fp_sensors,"(");

 if(flg_inte_quad == 1){
 	fprintf(fp_sensors," %f %f",intensity,angle_light);
 	//printf("intensity %f angle_light %f\n",intensity,angle_light);
 }

 for(j=0;j<num_sensors;j++){
        fprintf(fp_sensors," %f",observations.sensors[j]);
        //printf("write sensors[%d] %f\n",j,observations.sensors[j]);
 }

 fprintf(fp_sensors," ) \n");


 return(1);
}




float magnitude(coord vector){

        float magnitude;

        magnitude=(float)sqrt((vector.xc*vector.xc)+(vector.yc*vector.yc));
        //printf("magnitude: %f\n",magnitude);
        return(magnitude);
}




coord dif_vectors(coord vector1,coord vector2){

        coord dif;

        dif.xc=vector1.xc-vector2.xc;
        dif.yc=vector1.yc-vector2.yc;
        return(dif);
}

// This function is used to calculate the rotation angle for the Mvto command
float get_angle(float ang,float c,float d,float X,float Y){
        float x,y;

        x=c-X;
        y=d-Y;
        if((x == 0) && (y == 0)) return(0);
        if(fabs(x)<0.0001)      return((float) ((y<0.0f)? 3*PI/2  : PI/2) - ang );
        else{
                if(x>=0.0f&&y>=0.0f) return( (float)(atan(y/x)-ang) );
                else if(x< 0.0f&&y>=0.0f) return( (float)(atan(y/x)+PI-ang) );
                else if(x< 0.0f&&y<0.0f) return( (float)(atan(y/x)+PI-ang) );
                else return( (float)(atan(y/x)+2*PI-ang));
        }
}



void get_intensity_angle(coord coord_robot,coord coord_destination, float *intensity, float *light_angle){

 coord attraction_force;
 float mag;
 coord dif;
 float theta;
 float angle;


 theta=coord_robot.anglec;

 dif=dif_vectors(coord_robot,coord_destination);

 angle=get_angle(0,0,0,dif.xc,dif.yc);
 angle=angle - theta;
 if(angle < 0) angle = 2*PI + angle;
#ifdef DEBUG
 printf("light source angle %f\n",angle);
#endif
 *light_angle = angle;

 attraction_force=dif_vectors(coord_robot,coord_destination);
 mag=magnitude(attraction_force);
 *intensity=1/(mag*mag);
 
}




 // It quantizes the intensity 
//int quantize_intensity(coord coord_robot,coord coor_destination,float intensity,int flg){
int quantize_intensity(float intensity,int flg){

 int value=0;
 coord attraction_force;
 float mag;

 //attraction_force=dif_vectors(coord_robot,coor_destination);
 //mag=magnitude(attraction_force);
 //*intensity=1/(mag*mag);
 //printf("intensity %f k_INTENSITY %f\n",mag,K_INTENSITY);

 mag= sqrt(1/intensity);
 
 if(flg == 0){
 	if(mag > K_INTENSITY){
        	value = 0;
 	}
 	else{
         	value = 1;
 	}
 }
 else{
 	if(mag > K_INTENSITY*4){
        	value = 0;
 	}
 	else if(mag > K_INTENSITY*3){
         	value = 1;
 	}
 	else if(mag > K_INTENSITY*2){
         	value = 2;
 	}
 	else{
        	value = 3;
 	}
 }


 return value;

}


 // It quantizes the intensity 
int quantize_intensity_vector(coord coord_robot,coord coor_destination,float *intensity){

 int value=0;
 coord attraction_force;
 float mag;

 attraction_force=dif_vectors(coord_robot,coor_destination);
 mag=magnitude(attraction_force);
 *intensity=1/(mag*mag);
 //printf("intensity %f k_INTENSITY %f\n",mag,K_INTENSITY);

 if(mag > K_INTENSITY*4){
        value = 0;
 }
 else if(mag > K_INTENSITY*3){
         value = 1;
 }
 else if(mag > K_INTENSITY*2){
         value = 2;
 }
 else{
	value = 3;
 }


 return value;

}



// it gets the range from the laser sensors
void get_average_sensor(Raw observations, int start, int end, float *average){
 int i;
 float sd=0;

 //printf("range %d %d\n",start,end); 
 for(i=start; i < end;i++){
        sd=sd + observations.sensors[i];
        //printf("%d %f %f\n",i,sd,observations.sensors[i]); 
 }

 *average = sd/(float)(end-start);
 //printf(" average %f\n\n",*average);
}


 // It quantizes the inputs
int quantize_inputs(Raw observations, int size_vectors, int flg, int size_quantizer, char *path){

 int value=0;
 int i;
 float left,right;
 int interval = size_vectors/2;
 static int num_centroids;
 static Centroid *centroids;
 static int flg_first = 1;
 char centroid_file[200];
 float dst;
 int ofst=-1;



 if(flg == 0){
 	get_average_sensor(observations,interval,size_vectors,&left);
 	//printf("left sensor %f\n",left);
 	get_average_sensor(observations,0,interval,&right);
 	//printf("right sensor %f\n",right);

 	if( left < THRS_SENSOR) value = (value << 1) + 1;
 	else value = (value << 1) + 0;

 	if( right < THRS_SENSOR) value = (value << 1) + 1;
 	else value = (value << 1) + 0;

 	//printf("value %d\n",value);
 	value = value & 0xFFFFFFFF;
 	//printf("value %x\n",value); 
 }

 return(value);

}


 // It quantizes the destination this later 2-1-11
//int quantize_destination(coord coord_robot,coord coord_destination, float light_angle, int flg){
int quantize_destination(float angle, int flg){

 int value=0;
 //float angle;
 coord dif;
 float theta;
 
 
 //theta=coord_robot.anglec;
 //printf("pose x %f y %f theta %f dest x %f y %f\n",coord_robot.xc,coord_robot.yc,theta,coord_destination.xc,coord_destination.yc);

 /*
 dif=dif_vectors(coord_robot,coord_destination);
 angle=get_angle(0,0,0,dif.xc,dif.yc);
 //angle=get_angle(0,0,0,coord_destination.xc,coord_destination.yc);
 //printf("angle %f\n",angle);
 angle=angle - theta;
 if(angle < 0) angle = 2*PI + angle;
 printf("light source angle %f\n",angle);
 *light_angle = angle;
 */

 if(flg==0){
 	if( angle < PI / 2) value = 3;
 	else if ( angle < PI ) value = 1;
 	else if( angle < 3*PI/2) value = 0;
 	else value = 2;
 }
 else{
        if( angle < PI / 4) value = 0;
        else if ( angle < PI/2 ) value = 1;
        else if( angle < 3*PI/4) value = 2;
        else if( angle < PI) value = 3;
        else if( angle < 5*PI/4) value = 4;
        else if( angle < 6*PI/4) value = 5;
        else if( angle < 7*PI/4) value = 6;
        else value = 7;
 }


 return value;

}




// It get the angle of the quantized source light
float inverse_quantize_destination(int value){

 float angle;

 if(value == 0)      angle = PI / 4;
 else if(value == 1) angle = PI/2;
 else if(value == 2) angle =3*PI/4;
 else if(value == 3) angle = PI;
 else if(value == 4) angle = 5*PI/4;
 else if(value == 5) angle = 6*PI/4;
 else if(value == 6) angle = 7*PI/4;
 else if(value == 7) angle = 0;
 else angle = 0;

 return angle;

}






coord divide_vector_scalar(coord vector1,float cnt){

        coord div;

        div.xc=vector1.xc/cnt;
        div.yc=vector1.yc/cnt;
        return(div);
}



// it gets an unit vector
coord get_unit_vector(Behavior Bvector){

 float mag;
 coord unit_vector,vector;

 unit_vector.xc=0;
 unit_vector.yc=0;

 vector.xc=Bvector.xc;
 vector.yc=Bvector.yc;

 mag=magnitude(vector);

 if(mag != 0)
        unit_vector=divide_vector_scalar(vector,mag);

 return unit_vector;

}


// it returns the distance to closest poligon from the robot
int shs_distance_obstacle(float x1, float y1, float x2, float y2,float dist_advance, int *indx, int flg){

 int index=0;
 float distance;


  index=0;
  robot_sensor.vertex[1].x=x1;
  robot_sensor.vertex[1].y=y1;
  robot_sensor.vertex[2].x=x2+0.0001;
  robot_sensor.vertex[2].y=y2+0.0001;

  //printf("check x1 %f y1 %f\n",robot_sensor.vertex[1].x,robot_sensor.vertex[1].y);
  ////printf("check x2 %f y2 %f\n",robot_sensor.vertex[2].x,robot_sensor.vertex[2].y);

  //m=(y2-y1)/(x2-x1);
  //b=(x2*y1-y2*x1)/(x2-x1);
  robot_sensor.line[1].m= (robot_sensor.vertex[2].y-robot_sensor.vertex[1].y)/
                        (robot_sensor.vertex[2].x-robot_sensor.vertex[1].x);
  robot_sensor.line[1].b= (robot_sensor.vertex[2].x*robot_sensor.vertex[1].y-
                         robot_sensor.vertex[2].y*robot_sensor.vertex[1].x)/
                        (robot_sensor.vertex[2].x-robot_sensor.vertex[1].x);




  // it finds the distance to the closest polygon, distance_polygon function is in ../simulator/simulation.h
  if(flg == 0)
  	distance=distance_polygon(num_polygons_wrl,polygons_wrl,robot_sensor,&index);
  else 
  	distance=distance_polygon(num_polygons_unk+1,polygons_unk,robot_sensor,&index);

  *indx = index;

//#ifdef DEBUG
  //printf("distance_polygon %d  polygon name %s distance %f\n",index,polygons_wrl[index].name,distance);
  //printf("** check distance closests %f distance advance %f **\n",distance,fabs(dist_advance));
//#endif


 if(distance > fabs(dist_advance)){
        return 0;
 }else{
        return 1;
 }

}



// it will move the robot the desire angle and distance
int  mvrobot(FILE *fpw,AdvanceAngle DistTheta,coord *coord_robot ){
 int flg=0;
 int flg_unk=0;
 float new_xmv,new_ymv,new_thetamv;
 float xc,yc;
 float speed;
 coord new_coord;
 float xmv,ymv,thetamv;
 float distance, angle1;
 int dummy;
 float cnt=3.0;
 int indx=0;
 int flag = 0;
 int flag_unk = 1;


 xmv=coord_robot->xc;
 ymv=coord_robot->yc;
 thetamv=coord_robot->anglec;

 angle1 = DistTheta.angle;
 distance = DistTheta.distance;

//#ifdef DEBUG
 //printf("before x:%f, y:%f,  rad:%f\n",coord_robot->xc,coord_robot->yc,coord_robot->anglec);
 //printf("Distance %f before angle1 %f\n",distance,angle1);
//#endif


 new_thetamv = thetamv + angle1;

 //if(distance > 0.){
 	new_xmv = xmv + (float) distance*(float)cos((float) (new_thetamv));
 	new_ymv = ymv + (float) distance*(float)sin((float) (new_thetamv));
 //}
 //else{
 	//new_xmv = xmv - (float) cnt*distance*(float)cos((float) (new_thetamv));
 	//new_ymv = ymv - (float) cnt*distance*(float)sin((float) (new_thetamv));
//}

// #ifdef DEBUG
// printf("new_thetamv %f\n",new_thetamv);
// printf("new_xmv %f new_ymv %f\n",new_xmv,new_ymv);
//#endif

 // it checks if the robot new position is inside an obstacle
 //flg = shs_distance_obstacle(xmv,ymv,new_xmv,new_ymv,.5,&indx);
 flg = shs_distance_obstacle(xmv,ymv,new_xmv,new_ymv,cnt*distance,&indx,0);
 if(num_polygons_unk > 0) 
	flg_unk = shs_distance_obstacle(xmv,ymv,new_xmv,new_ymv,cnt*distance,&indx,1);

//#ifdef DEBUG
 //printf("check inside flg %d x:%.3f, y:%.3f,  rad:%.3f polygon  %d polygon name %s\n",flg,new_xmv,new_ymv,new_thetamv,indx,polygons_wrl[indx].name);
//#endif

 //flg = check_inside_polygon(new_xmv,new_ymv,polygons_wrl,indx);

 flag = inside_polygon(num_polygons_wrl,polygons_wrl,new_xmv,new_ymv,&indx);
 //printf("num_polygons_unk %d\n",num_polygons_unk);
 if(num_polygons_unk > 0) 
 	flag_unk = inside_polygon(num_polygons_unk+1,polygons_unk,new_xmv,new_ymv,&indx);

 if(angle1 > 5.75f) angle1=- (angle1 - 5.75f) ;
 if(new_thetamv > 6.2832) new_thetamv = new_thetamv - (float) 6.2832;
 else if(new_thetamv < -0.0) new_thetamv = new_thetamv + (float) 6.2832;

 new_xmv = xmv + (float) distance*(float)cos((float) (new_thetamv));
 new_ymv = ymv + (float) distance*(float)sin((float) (new_thetamv));

 //printf("flag %d flg %d flag_unk %d flg_unk %d\n",flag,flg,flag_unk,flg_unk);
 if((flag==1 && flg == 0) && ( flag_unk==1 && flg_unk == 0)){
 	coord_robot->xc=new_xmv;
 	coord_robot->yc=new_ymv;
 	coord_robot->anglec=new_thetamv;
 }
 else{
#ifdef DEBUG
 	printf("robot inside polygon %d x:%.3f, y:%.3f,  rad:%.3f polygon  %d polygon name %s\n",flag,new_xmv,new_ymv,new_thetamv,indx,polygons_wrl[indx].name);
        printf("The robot remains with the previous position\n");
#endif
        //printf("The robot remains with the previous position\n");
	dummy=0;
	fprintf(fpw,"( collision obstacle )\n");
 }

//#ifdef DEBUG
 //printf("after x:%f, y:%f,  rad:%f\n",coord_robot->xc,coord_robot->yc,coord_robot->anglec);
//#endif

 return flg;

}


float distance(coord vector,coord vector1){

 float x,x1,y,y1;
 float distance;

 x=vector.xc;
 x1=vector1.xc;
 y=vector.yc;
 y1=vector1.yc;

 // it calculates the distance
 distance=(float)sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
 //printf("Distance: %f\n",distance);

 return(distance);
}


// it quantizes a robot output
AdvanceAngle quantize_output(float advance, float angle, float max_advance, float max_angle){

 AdvanceAngle output;

 if(advance >= 0){
	if(advance >= max_advance/2.0){
               	output.distance = max_advance;
	}
	else{
		output.distance = 0.0;
	}
 }
 else{
	if(fabs(advance) >= max_advance/2.0){
               	output.distance=-max_advance;
	}
	else{
		output.distance = 0.0;
	}
 }

 if(angle >= 0){
	if(angle < PI/2) {

//        if(angle >= max_angle/2.0){
                output.angle = max_angle;
        }
	else if(angle > 3*PI/2){
                output.angle = - max_angle;
        }
 }
 else{  
	if(fabs(angle) < PI/2){
        //if(fabs(angle) >= max_angle/2.0){
                output.angle=-max_angle;
        }
	else if(fabs(angle) > 3*PI/2){
                output.angle=max_angle;
        }
 }


 return(output);

}



/*
// it generates a robot's output
AdvanceAngle generate_output(int out,float advance, float angle){

  AdvanceAngle output;

  switch(out){

        case 0: // Stop
                output.distance=0.0f;
                output.angle=0.0f;
                printf("STOP\n");
                break;

        case 1: // Forward
                output.distance=advance;
                output.angle=0.0f;
                printf("FORWARD\n");
                break;

        case 2: // backward
                output.distance=-advance;
                output.angle=0.0f;
                printf("BACKWARD\n");
                break;

        case 3:// Turn left
                output.distance=0.0f;
                output.angle=angle;
                printf("LEFT\n");
                break;

	        case 4: // Turn right
                output.distance=0.0f;
                output.angle=-angle;
                printf("RIGHT\n");
                break;

        default:printf("Output %d not defined used ", out);
                output.distance=0.0f;
                output.angle=0.0f;
                break;
  }

  return(output);

}
*/


int get_index_range(float obs, float r1, float r2, float r3){

 int index = 0;

 if((obs < r3) & (obs >= r2)){
	index = 3;	
 }
 else if((obs < r2) & (obs >= r1)){
	index = 2;	
 }else if (obs < r1){
	index = 1;
 }

 return index;

}




//void get_source(int quantized_attraction, float x, float y, int *j, int *k){
void get_source(int quantized_attraction, int *j, int *k){

// Check how the quantized attraction value is generated in function quantize_destination in ..//utilities/utilities.h
#ifdef DEBUG
 printf("quantized destination %d \n",quantized_attraction);
#endif
 // j represents the raw and k the columns
// if((x > 0.0) && (y > 0.0)){
 if(quantized_attraction == 0){
		// E
        	*j=3;
        	*k=6;
#ifdef DEBUG
		printf("Light source in the E\n");
#endif
        	//coor_destination.xc= 0.0001;
        	//coor_destination.yc= 0.0001;
 }
 else if(quantized_attraction == 1){
        	// NE
        	*j=6;
        	*k=6;
#ifdef DEBUG
		printf("Light source in the NE\n");
#endif
        	//coor_destination.xc= 1.0001;
        	//coor_destination.yc= 1.0001;
 }
 else if(quantized_attraction == 2){
        	// N
        	*j=6;
        	*k=3;
#ifdef DEBUG
		printf("Light source in the N\n");
#endif
        	//coor_destination.xc= 0.0001;
        	//coor_destination.yc= 1.0001;
 }
 else if(quantized_attraction == 3){
        	// NW
        	*j=6;
        	*k=1;
#ifdef DEBUG
		printf("Light source in the NW\n");
#endif
        	//coor_destination.xc= -1.0001;
        	//coor_destination.yc= -1.0001;
 }
 else if(quantized_attraction == 4){
        	// W
        	*j=3;
        	*k=1;
#ifdef DEBUG
		printf("Light source in the W\n");
#endif
        	//coor_destination.xc= -1.0001;
        	//coor_destination.yc= 0.0001;
 }
 else if(quantized_attraction == 5){
        	// SW
        	*j=1;
        	*k=1;
#ifdef DEBUG
		printf("Light source in the SW\n");
#endif
        	//coor_destination.xc= -1.0001;
        	//coor_destination.yc= -1.0001;
}
else if(quantized_attraction == 6){
        	// S
        	*j=1;
        	*k=3;
#ifdef DEBUG
		printf("Light source in the S\n");
#endif
        	//coor_destination.xc= 0.0001;
        	//coor_destination.yc= -1.0001;
}
else if(quantized_attraction == 7){
        	// SE
        	*j=1;
        	*k=6;
#ifdef DEBUG
		printf("Light source in the SE\n");
#endif
        	//coor_destination.xc= 1.0001;
        	//coor_destination.yc= -1.0001;
}
else{
		*j=6;
        	*k=6;
#ifdef DEBUG
        	printf("Light source not found\n");
#endif
}


 /*else if((x < 0.0) && (y > 0.0)){
        if(quantized_attraction == 0){
                // E (N)
                *j=6;
                *k=3;
                printf("Light source in the E\n");
                //coor_destination.xc= 0.0001;
                //coor_destination.yc= 0.0001;
        }
        else if(quantized_attraction == 1){
                // NE (SE)
                *j=1;
                *k=6;
                printf("Light source in the NE\n");
                //coor_destination.xc= 1.0001;
                //coor_destination.yc= 1.0001;
        }
        else if(quantized_attraction == 2){
                // N (W)
                *j=3;
                *k=1;
                printf("Light source in the N\n");
                //coor_destination.xc= 0.0001;
                //coor_destination.yc= 1.0001;
        }
        else if(quantized_attraction == 3){
                // NW (SW)
                *j=1;
                *k=1;
                printf("Light source in the NW\n");
                //coor_destination.xc= -1.0001;
                //coor_destination.yc= -1.0001;
        }
        else if(quantized_attraction == 4){
                // W (S)
                *j=1;
                *k=3;
                printf("Light source in the W\n");
                //coor_destination.xc= -1.0001;
                //coor_destination.yc= 0.0001;
        }
        if(quantized_attraction == 5){
                // SW (SE)
                *j=1;
                *k=6;
                printf("Light source in the SW\n");
                //coor_destination.xc= -1.0001;
                //coor_destination.yc= -1.0001;
        }
        else if(quantized_attraction == 6){
                // S (E)
                *j=3;
                *k=6;
                printf("Light source in the S\n");
                //coor_destination.xc= 0.0001;
                //coor_destination.yc= -1.0001;
        }
        else if(quantized_attraction == 7){
                // SE (NE)
                *j=6;
                *k=6;
                printf("Light source in the SE\n");
                //coor_destination.xc= 1.0001;
                //coor_destination.yc= -1.0001;
        }
 }
*/




}



void get_source_old(int attraction, float x, float y, int *j, int *k){

	printf(" 1 attraction %d x %f y %f\n",attraction,x,y);

 if((x > 0.0) && (y > 0.0)){
   switch(attraction){

        case 0: // SW
                *j=1;
                *k=1;
                break;

        case 1: // SE
                *j=6;
                *k=1;
                break;

        case 2: // NE
                *j=1;
                *k=6;
                break;

        case 3: // NW
                *j=6;
                *k=6;
                break;

        default:printf("attraction %d not defined\n", attraction);
                *j=6;
                *k=6;
                break;
    }
 }
 else if((x < 0.0) && (y > 0.0)){
   switch(attraction){

        case 0: // SW
                *j=1;
                *k=6;
                break;

        case 1: // SE
                *j=1;
                *k=1;
                break;

        case 2: // NE
                *j=6;
                *k=6;
                break;

        case 3: // NW
                *j=6;
                *k=1;
                break;

        default:printf("attraction %d not defined\n", attraction);
                *j=6;
                *k=6;
                break;
    }
 }
 else if((x < 0.0) && (y < 0.0)){
	printf("attraction %d x %f y %f\n",attraction,x,y);
   switch(attraction){

        case 0: // SW
                *j=6;
                *k=6;
                break;

        case 1: // SE
                *j=1;
                *k=6;
                break;

        case 2: // NE
                *j=6;
                *k=1;
                break;

        case 3: // NW
                *j=1;
                *k=1;
                break;

        default:printf("attraction %d not defined\n", attraction);
                *j=6;
                *k=6;
                break;
    }
 printf(" *j %d *k %d\n",*j,*k);
 }
 else if((x > 0.0) && (y < 0.0)){
   switch(attraction){

        case 0: // SW
                *j=6;
                *k=1;
                break;

        case 1: // SE
                *j=6;
                *k=6;
                break;

        case 2: // NE
                *j=1;
                *k=1;
                break;

        case 3: // NW
                *j=1;
                *k=6;
                break;

        default:printf("attraction %d not defined\n", attraction);
                *j=6;
                *k=6;
                break;
    }
 }
 else{
	*j=6;
	*k=6;
	printf("attraction not defined x %f y %f\n",x,y);
 }

 printf(" *j %d *k %d\n",*j,*k);

}



//#define DEBUG1 1
//#define DEBUG2 1

void ocupancy_grid_mdp(Raw observations, int size, int attraction, struct mdp_database *mdp, float range_sensor, float *rewards){

  int i,j,jj,k;
  float obs,r1,r2,r3;
  int index;
  float x,y;
  int type_0=TYPE_0;



  // It initializes the MDP occupancy grid
  for(j=1,jj=mdp->num_rows;j<=mdp->num_rows;j++,jj--){
        for(k=1;k<=mdp->num_columns;k++){
                mdp->type[jj][k]=1;
                //printf("type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
                //mdp->reward[jj][k]=-0.040000;
                mdp->reward[jj][k]=-rewards[1];
                //printf("grid reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
        }
  }

#ifdef DEBUG
  for(i=0; i < size;i++){
        printf("%d %f\n",i,observations.sensors[i]); 
  }
#endif

  r1 = range_sensor/3.2;
  r2 = range_sensor/2.2;
  r3 = range_sensor/1.2;

#ifdef DEBUG
  printf("range sensor %f\n",range_sensor);
  printf("range sensor r1 %f r2 %f r3 %f\n",r1,r2,r3);
#endif

  obs= observations.sensors[0];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[0] %f\n",observations.sensors[0]);
#endif
        jj = 3 - index+1;
        k = 3 + index;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[1];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[1] %f\n",observations.sensors[1]);
#endif
        jj = 3 - index+1;
        k = 3 + index;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }


  obs= observations.sensors[2];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[2] %f\n",observations.sensors[2]); 
#endif
        jj = 3 - index+2;
        k = 3 + index+1;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[3];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[3] %f\n",observations.sensors[3]); 
#endif
        jj = 3 - index+2;
        k = 3 + index+1;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[4];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[4] %f\n",observations.sensors[4]); 
#endif
        jj = 3 ;
        k = 3 + index;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[5];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[5] %f\n",observations.sensors[5]); 
#endif
        jj = 3 ;
        k = 3 + index;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }
 
  obs= observations.sensors[6];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[6] %f\n",observations.sensors[6]); 
#endif
        jj = 3 + index;
        k = 3 + index;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[7];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[7] %f\n",observations.sensors[7]); 
#endif
        jj = 3 + index ;
        k = 3;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[8];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[8] %f\n",observations.sensors[8]);
#endif
        jj = 3;
        k = 3 - index +1;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[9];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[9] %f\n",observations.sensors[9]);
#endif
        jj = 3;
        k = 3 - index +1;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[10];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[10] %f\n",observations.sensors[10]);
#endif
        jj = 3 - index +1;
        k = 3 - index +1;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[11];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[11] %f\n",observations.sensors[11]);
#endif
        jj = 3 ;
        k = 3 - index +1;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[12];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[12] %f\n",observations.sensors[12]);
#endif
        jj = 3;
        k = 3-index+1;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[13];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[13] %f\n",observations.sensors[13]);
#endif
        jj = 3 - index +1;
        k = 3 - index +1;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[14];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[14] %f\n",observations.sensors[14]);
#endif

        jj = 3 - index +1;
        k = 3 + index -2;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }

  obs= observations.sensors[15];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
#ifdef DEBUG
        printf("obs[15] %f\n",observations.sensors[15]);
#endif
        jj = 3 - index +1;
        k = 3 + index -2;
        //mdp->type[jj][k]=0;
        mdp->type[jj][k]=type_0;
#ifdef DEBUG1
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
#endif
        //mdp->reward[jj][k]=-1.00000;
        mdp->reward[jj][k]=-rewards[0];
#ifdef DEBUG1
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
#endif
  }




#ifdef DEBUG2
 printf("Quadrant attractor %d\n",attraction);
#endif

 get_source(attraction,&j,&k);

 mdp->type[j][k]=2;
#ifdef DEBUG2
 printf("type[%d][%d] %d\n",j,k,mdp->type[j][k]);
#endif
 //mdp->reward[j][k]=2.04;
 mdp->reward[j][k]=rewards[2];
#ifdef DEBUG2
 printf("reward[%d][%d] %f\n",j,k,mdp->reward[j][k]);
#endif


}




void ocupancy_grid_mdp_old(Raw observations, int size, int attraction, struct mdp_database *mdp, float range_sensor){

 int i,j,jj,k;
 float obs,r1,r2,r3;
 int index;
 float x,y;


 // It initializes the MDP occupancy grid
 for(j=1,jj=mdp->num_rows;j<=mdp->num_rows;j++,jj--){
        for(k=1;k<=mdp->num_columns;k++){
                mdp->type[jj][k]=1;
                //printf("type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
                mdp->reward[jj][k]=-0.040000;
                //printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
        }
 }


 for(i=0; i < size;i++){
        printf("%d %f\n",i,observations.sensors[i]);
 }

 r1 = range_sensor/3.0;
 r2 = range_sensor/2.0;
 r3 = range_sensor;

 printf("range sensor %f\n",range_sensor);

 if(size < 3){
   obs= observations.sensors[0];
   index = get_index_range(obs,r1,r2,r3);
   if(index != 0){
        printf("obs[0] %f\n",observations.sensors[0]);
        jj = 3 + index;
        k = 3 + index;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
   }

   obs= observations.sensors[1];
   index = get_index_range(obs,r1,r2,r3);
   if(index != 0){
        printf("obs[1] %f\n",observations.sensors[1]);
        jj = 3 - index;
        k = 3 + index-1;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
   }

 }
 else{
  obs= observations.sensors[0];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
        printf("obs[0] %f\n",observations.sensors[0]);
        jj = 3 ;
        k = 3 + index;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
  }

  obs= observations.sensors[1];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
        printf("obs[1] %f\n",observations.sensors[1]);
        jj = 3 + index-1;
        k = 3 + index-1;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
  }


  obs= observations.sensors[2];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
        printf("obs[2] %f\n",observations.sensors[2]);
        jj = 3 + index-1;
        k = 3;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
  }

  obs= observations.sensors[3];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
        printf("obs[3] %f\n",observations.sensors[3]);
        jj = 3 - index +1;
        k = 3;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
  }

  obs= observations.sensors[4];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
        printf("obs[4] %f\n",observations.sensors[4]);
        jj = 3;
        k = 3 - index +1;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
  }

  obs= observations.sensors[5];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
        printf("obs[5] %f\n",observations.sensors[5]);
        jj = 3 - index +1;
        k = 3 - index +1;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
  }

  obs= observations.sensors[6];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
        printf("obs[6] %f\n",observations.sensors[6]);
        jj = 3 - index +1;
        k = 3;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
  }

  obs= observations.sensors[7];
  index = get_index_range(obs,r1,r2,r3);
  if(index != 0){
        printf("obs[7] %f\n",observations.sensors[7]);
        jj = 3 - index +1;
        k = 3 + index -1;
        mdp->type[jj][k]=0;
        printf("obstacle type[%d][%d] %d\n",jj,k,mdp->type[jj][k]);
        mdp->reward[jj][k]=-1.00000;
        printf("reward[%d][%d] %f\n",jj,k,mdp->reward[jj][k]);
  }

 }



 printf("Quadrant attractor %d\n",attraction);
//here

/* 
 x = sqrt(2)* cos(PI/4 + angle);
 y = sqrt(2)* sin(PI/4 + angle);

 printf("x %f y %f\n",x,y);
 get_source(attraction,x,y,&j,&k);
 
*/

 get_source(attraction,&j,&k);


 mdp->type[j][k]=2;
 printf("type[%d][%d] %d\n",j,k,mdp->type[j][k]);
 mdp->reward[j][k]=2.04;
 printf("reward[%d][%d] %f\n",j,k,mdp->reward[j][k]);


}





/* Function that reads the mdp*/
void write_mdps(struct mdp_database mdp, char *path)
{
   FILE *fp2;
   int i,j,k,jj;
   char input_file[300];


   /* It opens the mdp file */
   sprintf(input_file,"%s%s_grid.mdp",path,mdp.objects_file);
#ifdef DEBUG
   printf("writting MDP FILE: %s\n",input_file);
#endif
   fp2=fopen(input_file,"w");
   if(fp2==0){
        printf("\n File %s can not be open",input_file);
        exit(0);
   }

   /* It writes the number of rows and columns */
   fprintf(fp2,"%d %d\n",mdp.num_rows,mdp.num_columns);


   // It writes the type and the rewards of the MDP celds 
   for(j=1,jj=mdp.num_rows;j<=mdp.num_rows;j++,jj--){
        for(k=1;k<=mdp.num_columns;k++){
                fprintf(fp2,"%d ",mdp.type[jj][k]);
                //printf("type[%d][%d] %d ",jj,k,mdp.type[jj][k]);
                fprintf(fp2,"%f ",mdp.reward[jj][k]);
                //printf("reward[%d][%d] %f ",jj,k,mdp.reward[jj][k]);
        }
	fprintf(fp2,"\n");
	//printf("\n");
   }


   // It prints the probabilities 
   for(i=1;i<=8;i++){ 
    for(j=1;j<=ROW_PRB;j++){
        for(k=1;k<=CLM_PRB;k++){
                fprintf(fp2,"%f ",mdp.pr[i][j][k]);
                //printf("pr[%d][%d][%d] %f\n",i,j,k,mdp.pr[i][j][k]);
        }
	fprintf(fp2,"\n");
    }
   }


   fclose(fp2); 

}

//Function created by me (1)
AdvanceAngle MoveRobot(float advance, float angle)
{
        AdvanceAngle output;
        output.distance = advance;
        output.angle = angle;

        return(output);
}

//Function created by me (2)
coord unitaryVector(coord inputVector)
{
        float magnitude; coord uVector = {0.0f, 0.0f, 0.0f};

        magnitude = sqrt(pow(inputVector.xc, 2) + pow(inputVector.yc, 2));
        uVector.xc = inputVector.xc / magnitude;
        uVector.yc = inputVector.yc / magnitude;
        uVector.anglec = atan2(inputVector.yc, inputVector.xc) * 180/PI;

        return uVector; 
}

//It generates a robot's output
AdvanceAngle generate_output(int out,float advance, float angle)
{
    AdvanceAngle output;

    switch(out){
        case 0: // Stop
                output.distance=0.0f;
                output.angle=0.0f;
                #ifdef DEBUG
                        printf("STOP\n");
                #endif
                break;
        case 1: // Forward
                output.distance=advance;
                output.angle=0.0f;
                #ifdef DEBUG
                        printf("FORWARD\n");
                #endif
                break;
        case 2: // backward
                output.distance=-advance;
                output.angle=0.0f;
                #ifdef DEBUG
                        printf("BACKWARD\n");
                #endif
                break;
        case 3:// Turn left
                output.distance=0.0f;
                output.angle=angle;
                #ifdef DEBUG
                        printf("LEFT\n");
                #endif
                break;
        case 4: // Turn right
                output.distance=0.0f;
                output.angle=-angle;
                #ifdef DEBUG
                        printf("RIGHT\n");
                #endif
                break;
	case 5: // Turn right_advance
                output.distance=advance;
                output.angle=-angle;
                #ifdef DEBUG
                        printf("RIGHT_ADVANCE\n");
                #endif
                break;
	case 6: // Turn left_advance
                output.distance=advance;
                output.angle=angle;
                #ifdef DEBUG
                        printf("LEFT_ADVANCE\n");
                #endif
                break;
	case 7: // Turn right_advance twice
                output.distance=advance;;
                output.angle=-2*angle;
                #ifdef DEBUG
                        printf("RIGHT_ADVANCE TWICE\n");
                #endif
                //output.distance=0.0f;
                //output.angle=0.0f;
                //printf("STOP\n");
                break;
        case 8: // Turn left_advance twice
                output.distance=advance;;
                output.angle=2*angle;
                printf("LEFT_ADVANCE TWICE\n");
                break;
        default:printf("Output %d not defined used ", out);
                output.distance=0.0f;
                output.angle=0.0f;
                printf("STOP\n");
                break;
  }
  return(output);
}




// it writes the sensors' vq readings to be plot by Tk
/*
int write_vq_obs_sensor(FILE *fpw, int index, char *sensor, int num_sensors, float start_angle, float range, int size_quantizer, char *path, int flg_ent_quad){
 int j;
 static int num_centroids;
 static Centroid *centroids;
 static int flg_first = 1;
 char centroid_file[100];
 int num_index;


 if(flg_first==1){

        // it allocates centroids space 
        if((centroids = (Centroid *)
                malloc( (NUM_MAX_CENTROIDS+1) * sizeof(Centroid))) == NULL){
                fprintf(stdout,"Allocation error: centroids");
                exit(0);
        }

        strcpy(centroid_file,CENTROID_FILE);

        // it reads the centroids 
        // function in ../vq/vq.h
        num_centroids= read_centroids(centroid_file,centroids,num_sensors,size_quantizer,path,FLG_VQ_SRT);

        flg_first=0;
   }


 num_index = index + 1;
 fprintf(fpw,"( sensor vq_%s %d %f %f",sensor,num_sensors,range,start_angle);

 for(j=1;j<=num_sensors;j++){
        fprintf(fpw," %f",centroids[num_index].sensors[j]);
#ifdef DEBUG
        printf("vq_laser[%d][%d] %f\n",num_index,j,centroids[num_index].sensors[j]);
#endif
 }

 fprintf(fpw," )\n");
#ifdef DEBUG
 printf("\n");
#endif

 return(1);
}

*/

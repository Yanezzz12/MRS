/*****************************************************************************
*                                                                            *
*  simulation.h                                                   	     *
*  ============                                                              *
*                                                                            *
*									     *
*                               J. Savage                                    *
*                               	                                     *
*                               FI-UNAM 2015                                 *
*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

//#define DBG_SIM 1 // Uncomment this line to see simulation debuging data
#define LARGEST_DISTANCE_SENSORS 5.0 // 5.0 meters
#define MAX_NUM_SENSORS 2048
#define MAX_NUM_RAW_SONAR 100
#define MAX_NUM_RAW_INFRARED 100
#define MAX_NUM_POLYGONS 200
#define NUM_MAX_VERTEX 10
#define STRSIZ 300
#define SIZE_LINE 10000
#define NUM_LASERS 682
#define INIT_ANGLE_LASER   (float) (((15.8f*PI*2)/360) - 2.3562f) // see Hokuyo laser manual for this
#define COVER_ANGLE  ((360.0f/1024.0f)*681.0f)




// *****************************************************************************
// * Structures.                                                               *
// *****************************************************************************

typedef struct _environment{
        int flag;
        char environment[100];
} Environment;


typedef struct _raw{
        int flag;
         int region;
         float x,y,theta[MAX_NUM_SENSORS],sensors[MAX_NUM_SENSORS];
} Raw;

typedef struct Vertex_ {
        float x;
        float y;
} Vertex;

typedef struct Line_ {
        float m;
        float b;
} Line;

typedef struct Polygon_ {
        char    name[STRSIZ];
        char    type[STRSIZ];
        int     num_vertex;
        Vertex  vertex[NUM_MAX_VERTEX];
        Line    line[NUM_MAX_VERTEX];
} Polygon;



// Global variables
Polygon polygons_wrl[MAX_NUM_POLYGONS],robot_sensor;
int num_polygons_wrl;
Polygon polygons_unk[MAX_NUM_POLYGONS];
int num_polygons_unk=0;



// it finds the elements of each of the polygon's lines
 // y=m*x-b
 // m=(y2-y1)/(x2-x1); b=(x2y1-y2x1)/(x2-x1)
void find_components_lines(int num_polygons,Polygon *polygons){
	int i,j;
	float m,b;
	float x1,y1,x2,y2;

	for(i=1;i<num_polygons;i++){

 		//printf("num.vertex %d\n",polygons_unk[i].num_vertex);
		for(j=1; j< polygons[i].num_vertex;j++){
			x1=polygons[i].vertex[j].x;
			x2=polygons[i].vertex[j+1].x+.00001f;
			y1=polygons[i].vertex[j].y;
			y2=polygons[i].vertex[j+1].y+.00001f;
			m=(y2-y1)/(x2-x1);
			b=(x2*y1-y2*x1)/(x2-x1);
			polygons[i].line[j].m=m;
			polygons[i].line[j].b=b;
			//printf("pol. %d name %s line %d m %f b %f \n",i,polygons[i].name,j,polygons[i].line[j].m, polygons[i].line[j].b);
		}
		// line of last and first vertex
		x1=polygons[i].vertex[j].x;
                x2=polygons[i].vertex[1].x+.00001f;
                y1=polygons[i].vertex[j].y;
                y2=polygons[i].vertex[1].y+.00001f;
                m=(y2-y1)/(x2-x1);
                b=(x2*y1-y2*x1)/(x2-x1);
                polygons[i].line[j].m=m;
                polygons[i].line[j].b=b;
		//printf("pol. %d name %s line %d m %f b %f \n",i,polygons[i].name,j,polygons[i].line[j].m, polygons[i].line[j].b);

	}

}


// it finds if the intersection points, xinter and yinter, are between a line
int interval_new(float x,float y,Polygon polygon,int j){
	
	float x1,x2,y1,y2;

	x2=polygon.vertex[j+1].x;
        x1=polygon.vertex[j].x;
	y1=polygon.vertex[j].y;
	y2=polygon.vertex[j+1].y;

	//printf("line interval %d x %f y %f\n",j,x,y);
	//printf("line interval %d x1 %f y1 %f\n",j,x1,y1);
	//printf("line interval %d x2 %f y2 %f\n",j,x2,y2);

	if((x1 < x2) && (y1 < y2)){
		if( ((x > x1) && (x < x2)) && ((y < y1) && (y < y2)) ) {
			printf("x %f y %f in the right side\n",x,y);
			return(1);
		}
	}
	else if((x1 > x2) && (y1 < y2)){
		if( (x > x2) && ((y >= y1) && (y <= y2)) ) {
			printf("x %f y %f in the right side\n",x,y);
			return(1);
		}
	}
	else if((x1 < x2) && (y1 > y2)){
		if( ((x > x1) && (x < x2)) && ((y < y1) && (y > y2)) ) return(1);
	}
	else if((x1 > x2) && (y1 > y2)){
		if( ((x < x1) && (x > x2)) && ((y > y1) && (y > y2)) ) return(1);
	}

	return(0);

}


// it finds if the intersection points, xinter and yinter, are between a line
int interval(float xinter,float yinter,Polygon polygon,int j){

        float x1,x2,y1,y2;

        if(polygon.vertex[j].x < polygon.vertex[j+1].x){
                x1=polygon.vertex[j].x;
                x2=polygon.vertex[j+1].x;
        }
        else{
                x2=polygon.vertex[j].x;
                x1=polygon.vertex[j+1].x;
        }
        if(polygon.vertex[j].y < polygon.vertex[j+1].y){
                y1=polygon.vertex[j].y;
                y2=polygon.vertex[j+1].y;
        }
        else{
                y2=polygon.vertex[j].y;
                y1=polygon.vertex[j+1].y;
        }

        //printf("line interval %d x1 %f y1 %f\n",j,x1,y1);
        //printf("line interval %d x2 %f y2 %f\n",j,x2,y2);
        //printf("line interval %d x %f y %f\n",j,xinter,yinter);


        if(x1 != x2){
                if( (xinter < x1) || (xinter > x2)) return(0);
        }
        /*else{
                if( xinter > x1) return(0);

        }*/
        if(y1 != y2){
                if( (yinter < y1) || (yinter > y2)) return(0);
        }
        /*else{

                if( yinter > y1) return(0);
        }*/


        return(1);

}



// it finds if the intersection points are between the robot's line
int interval_robot(float xinter,float yinter,Polygon robot){
                                                                               
        float x1,x2,y1,y2;
                                                                               
        if(robot.vertex[1].x < robot.vertex[2].x){
                x1=robot.vertex[1].x;
                x2=robot.vertex[2].x;
        }
        else{
                x2=robot.vertex[1].x;
                x1=robot.vertex[2].x;
        }
        if(robot.vertex[1].y < robot.vertex[2].y){
                y1=robot.vertex[1].y;
                y2=robot.vertex[2].y;
        }
        else{
                y2=robot.vertex[1].y;
                y1=robot.vertex[2].y;
        }
        //printf("line robot x1 %f y1 %f\n",x1,y1);
        //printf("line robot x2 %f y2 %f\n",x2,y2);
        
                                                                       
        if(x1 != x2)
        if( (xinter < x1) || (xinter > x2))
                return(0);
        if(y1 != y2)
        if( (yinter < y1) || (yinter > y2))
                return(0);
                                                                               
                                                                               
        return(1);
                                                                               
}

// it finds the distance between the robot and the intersection line
float magnitud(Polygon robot,float xinter,float yinter){
	
	float distance;
	float x,y;

	x = robot.vertex[1].x-xinter;
        y = robot.vertex[1].y-yinter;
	distance=sqrtf(x*x+y*y);

	return distance;
}



// it reads the file that conteins the environment description
int ReadPolygons(char *file,Polygon *polygons, float *dimx, float *dimy){

 FILE *fp;
 char data[STRSIZ];
 int i,num_poly=1;
 float tmp;
 int flg=0;
 float dimensions_room_x,dimensions_room_y;


 fp=fopen(file,"r");
 if(fp == NULL){
                printf("\nFile %s does not exists\n",file);
                exit(0);
 }

#ifdef DBG_SIM
 printf("World environment %s \n",file);
#endif

 while(fscanf(fp,"%s",data) != EOF){

    if(strcmp(";(",data) == 0){
	flg=1;
     	while(flg == 1){
          	fscanf(fp,"%s",data);
		sscanf(data,"%f",&tmp);
		if(strcmp(")",data) == 0) flg = 0;
	}
    }
    else if((strcmp("polygon",data) == 0) && (flg == 0) ){
          fscanf(fp,"%s",data);
          strcpy(polygons[num_poly].type,data);
          fscanf(fp,"%s",data);
          strcpy(polygons[num_poly].name,data);
          i=1;
	  flg=1;
     	  while(flg == 1){
		fscanf(fp,"%s",data);
		if(strcmp(")",data) == 0) {
                	polygons[num_poly].num_vertex=i-1;
			polygons[num_poly].vertex[i].x=polygons[num_poly].vertex[1].x; // to calculate intersecction range
                        polygons[num_poly].vertex[i].y=polygons[num_poly].vertex[1].y; // the first vertex its repeated on the last
                	num_poly++;
			flg = 0;
	        }
		else{
			sscanf(data,"%f",&tmp);
              		polygons[num_poly].vertex[i].x=tmp;
              		fscanf(fp,"%s",data);
              		sscanf(data,"%f",&tmp);
              		polygons[num_poly].vertex[i].y=tmp;
	 		//printf("polygon vertex %d x %f y %f\n",i,polygons[num_poly].vertex[i].x,polygons[num_poly].vertex[i].y);
              		i++;
        	}

	}
    }
    else if(strcmp("dimensions",data) == 0  && (flg == 0) ){
         fscanf(fp,"%s",data);
         fscanf(fp,"%s",data);
         sscanf(data,"%f",&dimensions_room_x);
         fscanf(fp,"%s",data);
         sscanf(data,"%f",&dimensions_room_y);
	 //printf("dimensions x %f y %f\n",dimensions_room_x,dimensions_room_y);
	 *dimx = dimensions_room_x;
	 *dimy = dimensions_room_y;
    }

 }

 fclose(fp);
 return num_poly;
}


// it finds if a point x,y is inside a polygon
int check_inside_polygon_new(float x,float y,Polygon *polygons,int indx){

 int i,j;
 int flg=0;
 float m,b;
 float tmp=0;
 int cnt=0;
 float kk[5] ;
 float smallest_x, smallest_y;
 float biggest_x, biggest_y;
 float x1,x2,y1,y2;

 i=indx;
 biggest_x = polygons[i].vertex[1].x;
 biggest_y = polygons[i].vertex[1].y;
 smallest_x = polygons[i].vertex[1].x;
 smallest_y = polygons[i].vertex[1].y;
 kk[1]=0.0;
 kk[2]=-1.0;
 kk[3]=-1.0;
 kk[4]=0.0;

 for(j=1; j<= polygons[i].num_vertex;j++){


        m=(y2-y1)/(x2-x1);
        b=(x2*y1-y2*x1)/(x2-x1);


        m=polygons[i].line[j].m;
        b=polygons[i].line[j].b;
        tmp = y - m*x - b;
	if((kk[j] - tmp) < 0 ){
                cnt++;
		if(polygons[i].vertex[j].x < smallest_x) smallest_x= polygons[i].vertex[j].x;
		else if(polygons[i].vertex[j].x > biggest_x) biggest_x= polygons[i].vertex[j].x;
		if(polygons[i].vertex[j].y < smallest_y) smallest_y= polygons[i].vertex[j].y;
		else if(polygons[i].vertex[j].y > biggest_y) biggest_y= polygons[i].vertex[j].y;
		#ifdef DBG_SIM
			printf("tmp %f\n",tmp);
                        printf("x %f y %f inside polygon %s vertex %d \n", x, y, polygons[i].name,j);
        		printf("x %f y %f polygon %s vertex %d tmp %f \n", x, y, polygons[i].name,j,tmp);
        		printf("vertex %d polygon_x1 %f polygon_x2 %f polygon_y1 %f polygon_y2 %f \n",
                                j,polygons[i].vertex[j].x,polygons[i].vertex[j+1].x,polygons[i].vertex[j].y,polygons[i].vertex[j+1].y);
        		printf("cnt %d smallest x %f smallest y %f biggest x %f biggest y %f\n",cnt,smallest_x,smallest_y,biggest_x,biggest_y);
		#endif
        }


 }

 if(polygons[i].num_vertex == cnt) {
	if( x >= smallest_x && x <= biggest_x && y >= smallest_y && y <= biggest_y ) {
		flg = 1;
	}		

 }

 return flg;

}



// it finds if a point x,y is inside a polygon
int check_inside_polygon(float x,float y,Polygon *polygons,int indx){

 int i,j;
 int flg=0;
 float m,b;
 float tmp=0;
 int cnt=0;
 float kk = 1.0;

 i=indx;

 for(j=1; j<= polygons[i].num_vertex;j++){

	m=polygons[i].line[j].m;
        b=polygons[i].line[j].b;
	tmp = y - m*x - b;
#ifdef DBG_SIM
  	printf("x %f y %f polygon %s vertex %d tmp %f \n", x, y, polygons[i].name,j,tmp);
	printf("vertex %d polygon_x1 %f polygon_x2 %f polygon_y1 %f polygon_y2 %f \n",
				j,polygons[i].vertex[j].x,polygons[i].vertex[j+1].x,polygons[i].vertex[j].y,polygons[i].vertex[j+1].y);
#endif
	
	if(tmp < kk){
		if(interval(x,y,polygons[i],j)==1){

			cnt++;
#ifdef DBG_SIM
          		printf("x %f y %f inside polygon %s vertex %d \n", x, y, polygons[i].name,j);
#endif
		}
	}	
 }

 if(polygons[i].num_vertex == cnt) flg = 1;
 

 return flg;

}


// it finds the distance to the closest polygon
int inside_polygon(int num_polygons,Polygon *polygons,float x,float y, int *indx){

  int i,j;
  int flg=0;
  int flag = 1;


 for(i=1;i<num_polygons;i++){

#ifdef DBG_SIM
      	printf("Checking polygon %s \n", polygons[i].name);
#endif

	flg = check_inside_polygon(x,y,polygons,i);
	if(flg == 1){
	
		*indx = i;
		i= num_polygons;
		flag = 0;
	}
 }

 return flag;

}




// it finds the distance to the closest polygon
float distance_polygon(int num_polygons,Polygon *polygons,Polygon robot,
							int *index_smallest){

	int i,j;
        float m,b,mr,br;
	float smallest_distance=(float)(0x7fffffff); 
	float xinter,yinter,distance;

	mr=robot.line[1].m;
	br=robot.line[1].b;
	//printf("\n robot m %f b %f", mr,br);

	for(i=1;i<num_polygons;i++){
                        

#ifdef DBG_SIM
	  printf("polygon %s \n", polygons[i].name);
#endif

                                                        
          for(j=1; j<= polygons[i].num_vertex;j++){
	  		//printf("line obstacle %d \n", j);
                	m=polygons[i].line[j].m;
                	b=polygons[i].line[j].b;
			//printf("pol. %d line %d m %f b %f \n",i,j,polygons[i].line[j].m, polygons[i].line[j].b);
			////printf("line m %f b %f\n", m,b);
			//xinter= (float)(br-b)/(float)(m-mr);
			xinter= (float)(b-br)/(float)(mr-m);
			//yinter=m*xinter+b;
			yinter=mr*xinter+br;

			//printf("intersection %d xinter %f yinter %f\n", j,xinter,yinter);

			if(interval(xinter,yinter,polygons[i],j)==1)
				if(interval_robot(xinter,yinter,robot)==1){
				
					distance=magnitud(robot,xinter,yinter);
					//printf("correct interval %d dist %f smallest %f\n",j,distance,smallest_distance);
					if(distance<smallest_distance){
						smallest_distance=distance;
						*index_smallest=i;
						//printf("smallest %f\n",smallest_distance);
					}
					//printf("smallest 1 %f\n",smallest_distance);
				}
        		}
    }

//#ifdef DBG_SIM
    //printf("smallest distance %f index %i\n",smallest_distance,*index_smallest);
//#endif

    //if (*index_smallest == 0) smallest_distance = 0.0;

    return smallest_distance;
}



// it reads the environment
void read_environment(char *file,int debug,float *dimx,float *dimy)
{
 
 int i;                                                                            
 int j;
 float dimxx,dimyy;


/* it reads the polygons */
 strcpy(polygons_wrl[0].name,"NULL");

#ifdef DBG_SIM
 if(debug == 1) printf("\nEnvironment file: %s\n",file);
#endif

 num_polygons_wrl=ReadPolygons(file,polygons_wrl,&dimxx,&dimyy);
 *dimx=dimxx;
 *dimy=dimyy;
                                                                               
 // it finds the elements of each of the polygon's lines
 // y=mx-b
 // m=(y2-y1)/(x2-x1); b=(x2y1-y2x1)/(x2-x1)
 find_components_lines(num_polygons_wrl,polygons_wrl);
                                                                               
// it prints the polygons
 if(debug == 1)
 for(i=1;i < num_polygons_wrl;i++){
        //printf("\npolygon[%d].name=%s\n",i,polygons_wrl[i].name);
        //printf("polygon[%d].type=%s\n",i,polygons_wrl[i].type);
                                                                               
        for(j=1; j<= polygons_wrl[i].num_vertex;j++){
          printf("polygon[%d].vertex[%d] x=%f y=%f\n",
                i,j,polygons_wrl[i].vertex[j].x,polygons_wrl[i].vertex[j].y);
          printf("polygon[%d].line[%d] m=%f b=%f\n",
                i,j,polygons_wrl[i].line[j].m,polygons_wrl[i].line[j].b);
                                                                               
        }
  
 }


}



// it returns a laser reading 
float shs_sensor(float *pq){

 int index;
 float distance;
 float angle;

  // it gets the sensor's position
  index=0;
  robot_sensor.vertex[1].x=pq[0];
  robot_sensor.vertex[1].y=pq[1];
  angle=pq[2];

  //printf("sensor pos. x %f y %f angle %f\n",pq[0],pq[1],pq[2]);


  robot_sensor.vertex[2].x=
                        SIZE_LINE*cosf(angle)+robot_sensor.vertex[1].x;
  robot_sensor.vertex[2].y=
                        SIZE_LINE*sinf(angle)+robot_sensor.vertex[1].y;
  


  //m=(y2-y1)/(x2-x1);
  //b=(x2*y1-y2*x1)/(x2-x1);
  robot_sensor.line[1].m= (robot_sensor.vertex[2].y-robot_sensor.vertex[1].y)/
                        (robot_sensor.vertex[2].x-robot_sensor.vertex[1].x);
  robot_sensor.line[1].b= (robot_sensor.vertex[2].x*robot_sensor.vertex[1].y-
                         robot_sensor.vertex[2].y*robot_sensor.vertex[1].x)/
                        (robot_sensor.vertex[2].x-robot_sensor.vertex[1].x);
   

  // it finds the distance to the closest polygon      
  distance=distance_polygon(num_polygons_wrl,polygons_wrl,robot_sensor,&index);
                                                                               

 //if(distance > LARGEST_DISTANCE_SENSORS)
	////distance=LARGEST_DISTANCE_SENSORS;
                                                                       
 return distance;
 
}


// it returns a laser reading 
float shs_sensor_unknown(float *pq){

 int index;
 float distance;
 float angle;

  // it gets the sensor's position
  index=0;
  robot_sensor.vertex[1].x=pq[0];
  robot_sensor.vertex[1].y=pq[1];
  angle=pq[2];

  //printf("sensor pos. x %f y %f angle %f\n",pq[0],pq[1],pq[2]);


  robot_sensor.vertex[2].x=
                        SIZE_LINE*cosf(angle)+robot_sensor.vertex[1].x;
  robot_sensor.vertex[2].y=
                        SIZE_LINE*sinf(angle)+robot_sensor.vertex[1].y;



  //m=(y2-y1)/(x2-x1);
  //b=(x2*y1-y2*x1)/(x2-x1);
  robot_sensor.line[1].m= (robot_sensor.vertex[2].y-robot_sensor.vertex[1].y)/
                        (robot_sensor.vertex[2].x-robot_sensor.vertex[1].x);
  robot_sensor.line[1].b= (robot_sensor.vertex[2].x*robot_sensor.vertex[1].y-
                         robot_sensor.vertex[2].y*robot_sensor.vertex[1].x)/
                        (robot_sensor.vertex[2].x-robot_sensor.vertex[1].x);


  // it finds the distance to the closest unknown polygon      
  distance=distance_polygon(num_polygons_unk+1,polygons_unk,robot_sensor,&index);


 //if(distance > LARGEST_DISTANCE_SENSORS)
        ////distance=LARGEST_DISTANCE_SENSORS;

 return distance;

}



// it selects the observations used
void select_obs(Raw observations, int num, Raw *sensor_vector, int num_sensors){
 int aux,pt,k;


 aux = (int) (num/num_sensors);
 for( k=1,pt=1; k<= num_sensors; k++,pt+=aux) {
          sensor_vector[0].sensors[k]=observations.sensors[pt];
          //printf("%d sensor(%d) = %f %f\n",k,pt,sensor_vector[0].sensors[k],observations.sensors[pt]);
 }

}


// it selects the angles of the sensors 
void select_angles(int num, float *Angles, float *angle_sensors, int num_sensors){
 int aux,pt,k;


 aux = (int) (num/num_sensors);
 for( k=1,pt=1; k<= num_sensors; k++,pt+=aux) {
          angle_sensors[k]=Angles[pt];
          //printf("%d angle sensor(%d) = %f \n",k,pt,angle_sensors[k]);
 }

}







// it generates simulated sensor values
void get_sensor_values(coord coord_robot,float start_angle,float range, Raw *sensor_vector, int num, float largest_value){

 int k;
 float distance;
 float pos[5];
 float angle;
 float inc_angle;
 float x, y, theta;


 x=coord_robot.xc;
 y=coord_robot.yc;
 theta=coord_robot.anglec;


 pos[0]=x;
 pos[1]=y;
 if(range == 1) inc_angle= range;
 else inc_angle = range /( num -1); 
 angle = start_angle;

 sensor_vector[0].x=x;
 sensor_vector[0].y=y;


 for( k=0; k< num; k++) {

 	  pos[2]=theta + k*inc_angle + start_angle;
	  //printf("k %d angle %f\n",k,pos[2]);
	  
          distance=shs_sensor(pos);
	  if(distance > largest_value) distance = largest_value;
          sensor_vector[0].sensors[k]=distance;
 	  sensor_vector[0].theta[k]=pos[2] - theta;
          //printf("simulator sensor(%d) = %f\n",k,sensor_vector[0].sensors[k]);
 }

}


// it generates simulated sensor values
void get_sensor_values_unknown(coord coord_robot,float start_angle,float range, Raw *sensor_vector, int num, float largest_value){

 int k;
 float distance;
 float pos[5];
 float angle;
 float inc_angle;
 float x, y, theta;


 x=coord_robot.xc;
 y=coord_robot.yc;
 theta=coord_robot.anglec;


 pos[0]=x;
 pos[1]=y;
 if(range == 1) inc_angle= range;
 else inc_angle = range /( num -1);
 angle = start_angle;

 sensor_vector[0].x=x;
 sensor_vector[0].y=y;


 for( k=0; k< num; k++) {

          pos[2]=theta + k*inc_angle + start_angle;
          //printf("k %d angle %f\n",k,pos[2]);

          distance=shs_sensor_unknown(pos);
          //printf("distance unknown sensor(%d) = %f\n",k,distance);
          if(distance > largest_value) distance = largest_value;
	  if(distance < sensor_vector[0].sensors[k]){ 
          	sensor_vector[0].sensors[k]=distance;
          	sensor_vector[0].theta[k]=pos[2] - theta;
#ifdef DBG_SIM
          	printf("simulator unknown sensor(%d) = %f\n",k,sensor_vector[0].sensors[k]);
#endif

	  }
 }

}

 //IT GENERATES RANDOM NUMBERS
float generaRR(float LMin,float LMax){
float n;
int BIG= 0x7FFFFFFF;

  n=LMin+ ((float) random()/(float) BIG ) *(LMax-LMin);
  return n;
}


#define SIZE_POL_UNK 6.0
#define NUM_STEPS_UNK 15
// it creates unknown obstacles ../simulator/simulation.h
void create_unknown_obstacles(FILE *fpw, float size_obs,float dimx, float dimy, int flg_delay, float pos_x, float pos_y){

 int i,j,k;
 int num_vertex=4;
 int n=NUM_STEPS_UNK;
 static float delta_tetha,tetha;
 static float x=0,y=0,hyp;
 static float xr=0,yr=0;
 static float xc[10],yc[10];
 static int flag=0;
 static float cntx[30], cnty[30];
 static int cnt=1;
 static char poly[31][400];
 static float inx,iny;
 int flg_inside;


 inx=1*dimx/n;
 iny=1*dimy/n;
 if(flag == 0) {

	// Initializes the random generator
	srandom((int)time((time_t *)NULL));
	if(num_polygons_unk > 30) num_polygons_unk = 30;
	k=1;
 	for(i=1;i<=num_polygons_unk;i++){

		if(k==1){
			xc[i]=generaRR(inx/2,dimx/2-inx/2);
			yc[i]=generaRR(iny/2,dimy/2-iny/2);
			cntx[i] = 0;
			cnty[i] = 1;
		}
		else if(k==2){
			xc[i]=generaRR(inx/2,dimx/2-inx/2);
                	yc[i]=generaRR(iny/2,dimy/2-iny/2);
			cntx[i] = 1;
			cnty[i] = 0;
		}
		else if(k==3){
			xc[i]=generaRR(inx/2,dimx/2);
                	yc[i]=generaRR(iny/2,dimy/2);
			cntx[i] = 1;
			cnty[i] = 1;
		}
		else if(k==4){
			xc[i]=generaRR(dimx/2,dimx);
                	yc[i]=generaRR(dimy/2,dimy);
			cntx[i] = -1;
			cnty[i] = -1;
		}
		else if(k==5){
                        xc[i]=generaRR(dimx/2,dimx-inx/2);
			yc[i]=generaRR(dimy/2,dimy-iny/2);
                        cntx[i] = 0;
                        cnty[i] = -1;
                }
		else if(k==6){
                        xc[i]=generaRR(dimx/2,dimx-inx/2);
			yc[i]=generaRR(dimy/2,dimy-iny/2);
                        cntx[i] = -1;
                        cnty[i] = 0;
                        k=1;
                }

		k++;
	
	}		

 	//delta_tetha= 2*PI/n;
 	delta_tetha= 2*PI/num_vertex;
 	hyp = SIZE_POL_UNK*size_obs;
	flag = 1;
 
 }

 if(flg_delay==1){
    for(i=1;i<=num_polygons_unk;i++){

	xr=generaRR(-0.01,0.01);
	yr=generaRR(-0.01,0.01);
        //inx=.1;
	//iny=.1;
 	xc[i]= xc[i] + cntx[i] * inx + xr;
 	yc[i]= yc[i] + cnty[i] * iny + yr;
   	strcpy(poly[i],"( unknown ");
 	polygons_unk[i].num_vertex = num_vertex;
	strcpy(polygons_unk[i].name,"unknown");
 	tetha = 2*PI - delta_tetha;
 	//tetha = 2*PI;

	for(j=1; j<= num_vertex;j++){
		x= (hyp * cos(tetha)) + xc[i];
                y= (hyp * sin(tetha)) + yc[i];

		polygons_unk[i].vertex[j].x= x;
		polygons_unk[i].vertex[j].y= y;
                sprintf(poly[i],"%s %f %f ",poly[i],polygons_unk[i].vertex[j].x,polygons_unk[i].vertex[j].y);
		tetha= tetha - delta_tetha;
	}
	polygons_unk[i].vertex[j].x= polygons_unk[i].vertex[1].x;
        polygons_unk[i].vertex[j].y= polygons_unk[i].vertex[1].y;
   
   }

 
   find_components_lines(num_polygons_unk+1,polygons_unk);

   for(i=1;i<=num_polygons_unk;i++){

        flg_inside = check_inside_polygon_new(pos_x,pos_y,polygons_unk,i);
	#ifdef DBG_SIM
        printf("Checking inside polygon unknown %s %d\n", polygons_unk[i].name,flg_inside);
	#endif
        if(flg_inside == 1){
 		tetha = 2*PI - delta_tetha;
   		strcpy(poly[i],"( unknown ");
		for(j=1; j<= num_vertex;j++){
			x= (hyp * cos(tetha)) + 10*xc[i];
                	y= (hyp * sin(tetha)) + 10*yc[i];

                	polygons_unk[i].vertex[j].x= x;
                	polygons_unk[i].vertex[j].y= y;
			tetha= tetha - delta_tetha;

                	sprintf(poly[i],"%s %f %f ",poly[i],polygons_unk[i].vertex[j].x,polygons_unk[i].vertex[j].y);
        	}
        	polygons_unk[i].vertex[j].x= polygons_unk[i].vertex[1].x;
        	polygons_unk[i].vertex[j].y= polygons_unk[i].vertex[1].y;

		// it saves the unknown obstacles' positions
		fprintf(fpw,"%s )\n",poly[i]);
		//printf("%s )\n",poly[i]);
        }
	else{
		// it saves the unknown obstacles' positions
        	fprintf(fpw,"%s )\n",poly[i]);

        	#ifdef DBG_SIM
        	printf("%d %s )\n",i,poly[i]);
        	#endif

	}
   }

   cnt++;

   if(cnt > n){
	cnt=1;
	flag=0;
   }

 }
 else{
 	for(i=1;i<=num_polygons_unk;i++){
   		// it saves the previous unknown obstacles' positions
   		fprintf(fpw,"%s )\n",poly[i]);
	}
 }


}




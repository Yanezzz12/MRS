/************************************************************************
*									*
*	random.h							*
*									*
*	This program generates Uniform or Gaussian random numbers.	*
*									*
*									*
*			J. Savage 					*
*			DEPFI-UNAM	11-2003				*
*									*
*************************************************************************/				
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define Lmin 0.0
#define Lmax 1.0
#define PERCENTAGE_CHANGE 0.10


// generates a uniform random variable
float generaR(float Min,float Max){
 float n;
 //int BIG= 0x7FFFFFFF;
 int BIG= RAND_MAX;

 n=Min+ ((float) random()/(float) BIG ) *(Max-Min);

 return n;
}



float random_gaussian(float mean, float variance,float *gaussian){
 float u1,u2,v1,v2,w,y,sqrt_variance;

 sqrt_variance=(float) sqrt(variance);

 while(1){

	// it generates a uniform random variable between 0 to 1
 	u1=generaR(Lmin,Lmax);
 	u2=generaR(Lmin,Lmax);

	//printf("u1 %f u2 %f",u1,u2);
 	v1=2*u1-1;
 	v2=2*u2-1;
 	w=v1*v1+v2*v2;

	//printf(" w %f \n",w);
	if(w < 1) {
		y=(float) sqrt( ((-2*log(w))/w));
		gaussian[1]=sqrt_variance*v1*y + mean;
		gaussian[2]=sqrt_variance*v2*y + mean;
 		return(1);
 	}
 }

}



// it adds noise to the sensors' values
void add_noise_obs(Raw *sensor_vector, float *intensity, float *angle_light, int num_sensors, char *path){

 int k;
 float noise;
 float gaussian[3];
 float tmp1,tmp2;
 int j;
 FILE *fp;
 char file[300];
 static int flg=0;
 static float mean,variance;
 static float lmin,lmax;
 static int type;
 static float mean_I,variance_I;
 static float lmin_I,lmax_I;
 static int type_I;
 static float mean_T,variance_T;
 static float lmin_T,lmax_T;
 static int type_T;



 if(flg==0){
         // Initializes the random generator
        srandom((int)time((time_t *)NULL));
        sprintf(file,"%srandom_settings_advance_angle_sensors.dat",path);
	/* random_settings_advance_angle_sensors.dat, 0 Uniform PDF; 1 Gaussian PDF
		0 -0.004 0.004  //advance
		0 -0.03927 0.03927 // angle
		1 0.0018 0.00018 // laser
		1 10.0 1.0  // light intensity
		0 -0.062832 0.062832 // angle light
	*/
        if((fp=fopen(file,"r")) == NULL){
                        printf("File %s can not be open\n",file);
                        exit(0);
        }
#ifdef DEBUG
        printf("Random settings file %s\n",file);
#endif
        // it reads the settings for the advance noise
        // it reads the settings for the angle noise
	for(j=1;j<7;j++){
        	fscanf(fp,"%f",&tmp1);
	}

	// it reads the settings for the sensors noise
        fscanf(fp,"%d",&type);
        fscanf(fp,"%f",&tmp1);
        fscanf(fp,"%f",&tmp2);
        if(type==0){
                lmin=tmp1;
                lmax=tmp2;
#ifdef DEBUG
		printf("noise lidar ranges lmin %f lmax %f\n",lmin,lmax);
#endif
        }
        else {
                mean=tmp1;
                variance=tmp2;
#ifdef DEBUG
		printf("lidar noise mean %f variance %f\n",mean,variance);
#endif
        }

	
	// it reads the settings for the intensity noise
        fscanf(fp,"%d",&type_I);
        fscanf(fp,"%f",&tmp1);
        fscanf(fp,"%f",&tmp2);
        if(type_I==0){
                lmin_I=tmp1;
                lmax_I=tmp2;
#ifdef DEBUG
		printf("Intensity lmin %f lmax %f\n",lmin_I,lmax_I);
#endif
        }
        else {
                mean_I=tmp1;
                variance_I=tmp2;
#ifdef DEBUG
		printf("Intensity mean %f variance %f\n",mean_I,variance_I);
#endif
        }

	// it reads the settings for the angle intensity noise
        fscanf(fp,"%d",&type_T);
        fscanf(fp,"%f",&tmp1);
        fscanf(fp,"%f",&tmp2);
        if(type_T==0){
                lmin_T=tmp1;
                lmax_T=tmp2;
#ifdef DEBUG
		printf("Angle Light lmin %f lmax %f\n",lmin_T,lmax_T);
#endif
        }
        else {
                mean_T=tmp1;
                variance_T=tmp2;
#ifdef DEBUG
		printf("Angle Light mean %f variance %f\n",mean_T,variance_T);
#endif

        }

        fclose(fp);
        flg=1;
 }

 
 //printf("random num_sensors %d\n",num_sensors);

 for( k=0; k< num_sensors; k++) {

	if(type==1) {
        
		// it generates a Gaussian random variable 
		random_gaussian(mean,variance,gaussian);
        	noise=gaussian[1];
 	}
 	else{
        	// it generates a Uniform random variable
        	noise=generaR(lmin,lmax);
 	}

	   
        //printf("sensor(%d) = %f \n",k,sensor_vector[0].sensors[k]);
	//printf("noise %f\n",noise);
        sensor_vector[0].sensors[k]=sensor_vector[0].sensors[k]+ noise;
        //printf("noise+sensor(%d) = %f \n",k,sensor_vector[0].sensors[k]);

 }


 if(type_I==1){
	// it generates a Gaussian random variable for the intensity 
        random_gaussian(mean_I,variance_I,gaussian);
        noise=gaussian[1];
 }
 else{
        // it generates a Uniform random variable
        noise=generaR(lmin_I,lmax_I);
 }
 *intensity = *intensity + noise; 

 if(*intensity <= 0) *intensity=0.1;

 if(type_T==1){
        // it generates a Gaussian random variable for the angle
        random_gaussian(mean_T,variance_T,gaussian);
        noise=gaussian[1];
 }
 else{
        // it generates a Uniform random variable
        noise=generaR(lmin_T,lmax_T);
 }
 *angle_light = *angle_light + noise;

 //if(*angle_light <= 0)*angle_light = 0.1;
 if(*angle_light <= 0)*angle_light = *angle_light + 7*PI/4;

}




void get_random_advance_angle(float *advance, float *angle, char *path)
{

 float gaussian[3],uniform;
 float tmp1,tmp2;
 int i,dist,num_random;
 FILE *fp;
 char file[300];
 static int flg=0;
 static float mean_a,variance_a;
 static float lmin_a,lmax_a;
 static float mean_t,variance_t;
 static float lmin_t,lmax_t;
 static int type_a,type_t;


 if(flg==0){
	 // Initializes the random generator
  	srandom((int)time((time_t *)NULL));
	sprintf(file,"%srandom_settings_advance_angle_sensors.dat",path);
	if((fp=fopen(file,"r")) == NULL){
                        printf("File %s can not be open\n",file);
                        exit(0);
       	}
#ifdef DEBUG
        printf("Random settings file %s\n",file);
#endif
	// it reads the settings for the advance noise
       	fscanf(fp,"%d",&type_a);
	fscanf(fp,"%f",&tmp1);
	fscanf(fp,"%f",&tmp2);
	if(type_a==0){
		lmin_a=tmp1;
		lmax_a=tmp2;
	}
	else {
		mean_a=tmp1;
		variance_a=tmp2;
	}
	// it reads the settings for the angle noise
       	fscanf(fp,"%d",&type_t);
	fscanf(fp,"%f",&tmp1);
	fscanf(fp,"%f",&tmp2);

	if(type_t==0){
		lmin_t=tmp1;
		lmax_t=tmp2;
	}
	else {
		mean_t=tmp1;
		variance_t=tmp2;
	}

 	fclose(fp);
	flg=1;
 }


 if(type_a==1) {
        // it generates a Gaussian random variable 
	random_gaussian(mean_a,variance_a,gaussian);
        //printf("Gaussian %f %f\n",gaussian[1],gaussian[2]);
	*advance=gaussian[1];
 }
 else{
	// it generates a Uniform random variable
        uniform=generaR(lmin_a,lmax_a);
        //printf("Uniform %f",uniform);
	*advance=uniform;
 }

 if(type_t==1) {
        // it generates a Gaussian random variable 
        random_gaussian(mean_t,variance_t,gaussian);
        //printf("Gaussian %f %f\n",gaussian[1],gaussian[2]);
        *angle=gaussian[1];
 }
 else{
        // it generates a Uniform random variable
        uniform=generaR(lmin_t,lmax_t);
        //printf("Uniform %f",uniform);
        *angle=uniform;
 }

 //printf("random noise advance %f angle %f\n",*advance,*angle);
 

}




float read_random_percentage(char *path){

 FILE *fp;
 char file[300];
 int j;
 float percentage = PERCENTAGE_CHANGE;
 float tmp1;
 int type;

 // Initializes the random generator
 srandom((int)time((time_t *)NULL));
 sprintf(file,"%srandom_settings_advance_angle_sensors.dat",path);
 if((fp=fopen(file,"r")) == NULL){
                        printf("File %s can not be open\n",file);
			return percentage;
 }
#ifdef DEBUG
 printf("Random settings file %s\n",file);
#endif
 // it reads the settings for the advance noise
 // it reads the settings for the angle noise
 for(j=1;j<7;j++){
        	fscanf(fp,"%f",&tmp1);
 }

 // it reads the settings for the sensors noise
 fscanf(fp,"%d",&type);
 fscanf(fp,"%f",&tmp1);
 fscanf(fp,"%f",&tmp1);
 fscanf(fp,"%f",&percentage);

 fclose(fp);

 return percentage;

}

 




int change_bits_random(int value,int num_bits, char *path){

 int new_value;
 int num_bits_mutation;
 int i,j,k;
 int mask;
 float random;
 static float percentage;
 static int flg=1;

 if(flg == 1){
 	percentage=read_random_percentage(path);
	flg=0;
 }

 new_value=value;
 random = generaR(0.0,1.0);
 //printf("random %f percentage %f\n",random,percentage);

 if(random > percentage) return (new_value);

 num_bits_mutation=num_bits;
 k= (int) generaR(0.0,num_bits_mutation);
 //printf("Max. num_bits_change %d num_bits_change %d\n",num_bits_mutation,k+1);

 for(j=0;j<= k; j++){
        i= (int) generaR(0.0,num_bits);
	mask =(1 << i);
	if( (new_value & mask ) == 0) new_value = new_value | mask;
	else {
		new_value = new_value & (~mask);
	} 
        //printf("num %d bit_changed %d\n",j,i);
 }

 return(new_value);

}




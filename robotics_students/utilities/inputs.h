

// it gets line inputs 
int get_inputs(int argc, char **argv, Inputs *inputs)
{

 int i;

 // It sets default values 
 inputs->xo=.10;
 inputs->yo=.10;
 inputs->angle_robot=0.0;
 inputs->xd=1.60;
 inputs->xd=1.60;
 strcpy(inputs->sensor,"laser");
 inputs->num_sensors=2;
 inputs->theta_sensor=-0.2;
 inputs->range_sensor=0.4;
 strcpy(inputs->environment,"room");
 strcpy(inputs->output_file,"room");
 strcpy(inputs->path,"../data/");
 inputs->radio_robot=RADIO_ROBOT;
 inputs->Mag_Advance=MAG_ADVANCE;
 inputs->max_angle=TURN_ANGLE;
 inputs->number_steps= LIMIT_SIM;
 inputs->selection = 1;
 inputs->largest_value = LARGEST_DISTANCE_SENSORS;
 inputs->noise = 1;
 inputs->omni = OMNI;
 inputs->vq = 0;
 inputs->size_vq = 4;
 inputs->pr_out = 0;
 inputs->nn_rec = 1;
 inputs->nn_unk = 0;


  // reads input parameters 
  if (argc>1) {
        for(i=1;i<argc;i++){
          if (!strcmp(argv[i],"-h")){

                printf("Sintax: %s -x x_origen -y y_origen -a angle_robot -v x_destination -z y_destination -s sensor -n num_sensors -t theta_sensor -r range_sensor -p path_data -e environment -radio radio_robot -advance magnitud_advance -MaxAngle max_angle -steps number_steps -largest largest_value_sensors -noise add_noise -selection selection -vq quantize_sensors -size_vq vector_quantizer_size -pr_out random_outputs -nn_rec recurrent_nn -nn_unk num_unknown_obstacles\n",argv[0]);
                printf("examples:\n");
                printf("%s -x 1 -y 2.5 -a 3.1416 -v 2.0 -z 3.1 -s laser -n 4 -t -0.7075 -r 3.1416 -selection 3\n",argv[0]);
		printf("%s -x 0.28 -y 0.795 -a 0.0 -v 0.6525 -z 0.7875 -s laser -n 2 -t -0.2000 -r 0.4000 -radio 0.03 -advance 0.04 -MaxAngle 0.3 -steps 100 -largest 0.3 -noise 1 -nn_unk 5\n",argv[0]);
		printf("%s -x 0.28 -y 0.795 -a 0.0 -v 0.6525 -z 0.7875 -s laser -n 2 -t -0.2000 -r 0.4000 -radio 0.03 -advance 0.04 -MaxAngle 0.3 -steps 100 -largest 0.3 -noise 1 -vq 0\n",argv[0]);
		printf("%s -x 0.28 -y 0.795 -a 0.0 -v 0.6525 -z 0.7875 -s laser -n 2 -t -0.2000 -r 0.4000 -radio 0.03 -advance 0.04 -MaxAngle 0.3 -steps 100 -largest 0.3 -noise 1 -vq 0\n",argv[0]);
		printf("%s -x 0.28 -y 0.795 -a 0.0 -v 0.6525 -z 0.7875 -s laser -n 2 -t -0.2000 -r 0.4000 -radio 0.03 -advance 0.04 -MaxAngle 0.3 -steps 100 -largest 0.3 -noise 1 -vq 1 -size_vq 8 -selection 11 -pr_out 1 -nn_rec 1\n",argv[0]);
		printf("%s -x 0.222467 -y 0.288354 -a 1.451600 -v 0.607500 -z 0.725000 -s laser -n 16 -t -2.356100 -r 4.712200 -p ../test_data_14/ -e random -advance 0.040000 -MaxAngle 0.785400 -selection 9 -largest 0.180000 -noise 1 -omni 1 -vq 1 -size_vq 8 -pr_out 1 -out_file random_fsm\n",argv[0]);
                exit(1);
          }
	  else if (!strcmp(argv[i],"-a")){
                inputs->angle_robot=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-e")){
                strcpy(inputs->environment,argv[i+1]);
          }
          else if (!strcmp(argv[i],"-out_file")){
                strcpy(inputs->output_file,argv[i+1]);
          }
          else if (!strcmp(argv[i],"-n")){
                inputs->num_sensors=atoi(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-p")){
                strcpy(inputs->path,argv[i+1]);
          }
          else if (!strcmp(argv[i],"-advance")){
                inputs->Mag_Advance=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-r")){
                inputs->range_sensor=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-radio")){
                inputs->radio_robot=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-s")){
                strcpy(inputs->sensor,argv[i+1]);
          }
          else if (!strcmp(argv[i],"-t")){
                inputs->theta_sensor=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-v")){
                inputs->xd=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-x")){
                inputs->xo=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-y")){
                inputs->yo=atof(argv[i+1]);
          }
	  else if (!strcmp(argv[i],"-z")){
                inputs->yd=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-MaxAngle")){
                inputs->max_angle=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-steps")){
                inputs->number_steps=atoi(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-selection")){
                inputs->selection=atoi(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-largest")){
                inputs->largest_value=atof(argv[i+1]);
          }
          else if (!strcmp(argv[i],"-noise")){
                inputs->noise=atoi(argv[i+1]);
          }
	  else if (!strcmp(argv[i],"-omni")){
                inputs->omni=atoi(argv[i+1]);
          }
	  else if (!strcmp(argv[i],"-vq")){
                inputs->vq=atoi(argv[i+1]);
          }
	  else if (!strcmp(argv[i],"-size_vq")){
                inputs->size_vq=atoi(argv[i+1]);
          }
	  else if (!strcmp(argv[i],"-pr_out")){
                inputs->pr_out=atoi(argv[i+1]);
          }
	  else if (!strcmp(argv[i],"-nn_rec")){
                inputs->nn_rec=atoi(argv[i+1]);
          }
	  else if (!strcmp(argv[i],"-nn_unk")){
                inputs->nn_unk=atoi(argv[i+1]);
          }
    }
  }
  else{

      printf("\nIt will use default values:\n");
  }

  printf("%s -x %f -y %f -a %f -v %f -z %f -s %s -n %d -t %f -r %f -p %s -e %s -advance %f -MaxAngle %f -selection %d -largest %f -noise %d -omni %d -vq %d -size_vq %d -pr_out %d -out_file %s -nn_rec %d -nn_unk %d\n",
	argv[0],inputs->xo,inputs->yo,inputs->angle_robot,inputs->xd,inputs->yd,inputs->sensor,inputs->num_sensors,
	inputs->theta_sensor,inputs->range_sensor, inputs->path,inputs->environment,inputs->Mag_Advance,
	inputs->max_angle,inputs->selection,inputs->largest_value,inputs->noise, inputs->omni, inputs->vq, inputs->size_vq,
	inputs->pr_out,inputs->output_file,inputs->nn_rec, inputs->nn_unk);



  return(0);

}




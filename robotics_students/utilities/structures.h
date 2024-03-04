
// *****************************************************************************
// * Structures.                                                               *
// *****************************************************************************

typedef struct _coord{
        float xc,yc,anglec;
} coord;

typedef struct _Behavior{
        float xc,yc,status;
        int flg_vector;
} Behavior;

typedef struct _AdvanceAngle{
        float distance,angle;
} AdvanceAngle;


typedef struct _Inputs{

 char path[256];
 char file[256];
 char environment[100];
 char sensor[100];
 float xo,yo,xd,yd;
 float angle_robot;
 float theta_sensor;
 int num_sensors;
 float range_sensor;
 int num_vectors;
 float Mag_Advance;
 float radio_robot;
 float max_angle;
 int number_steps;
 int selection;
 float largest_value;
 int noise;
 int omni;
 int vq;
 int size_vq;
 int pr_out;
 char output_file[256];
 int nn_rec;
 int nn_unk;
} Inputs;

#define MAX_TYPE 20
#define ROW_PRB 3
#define CLM_PRB 3
#define ROW_RWD 6
#define CLM_RWD 6
#define NUM_MDP_PRB 8



struct mdp_database {
        char objects_file[300];
        int type[MAX_TYPE][MAX_TYPE];
        float reward[MAX_TYPE][MAX_TYPE];
        char policy[MAX_TYPE][MAX_TYPE];
        float pr[9][ROW_PRB+1][CLM_PRB+1];
        int num_rows;
        int num_columns;
	float reward_0;
	float reward_1;
	float reward_2;
	float gamma;
};

struct table {
 int x[4][4];
 int y[4][4];
};


#define NUM_MAX_SENSORS 512
#define NUM_MAX_CENTROIDS 1024


typedef struct _centroid{
        int num;
        float x,y,theta,sensors[NUM_MAX_SENSORS];
} Centroid;


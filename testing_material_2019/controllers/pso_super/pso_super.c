#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "pso.h"
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define ROBOTS 1
#define MAX_ROB 5
#define ROB_RAD 0.035
#define ARENA_SIZE .15
#define MAX_DIST 3.5
#define MAX_SENS 500
#define SENSOR_RANGE 0.5
#define V_MAX       0.1287              // [m/s] e-puck maximum speed
#define TIME_STEP	64		// [ms] Length of time step
#define DESIRED_CM     0.15

#define NB_SENSOR 8                     // Number of proximity sensors

/* PSO definitions */
#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 0.5                     // Weight of attraction to personal best
#define NBWEIGHT 0.5                    // Weight of attraction to neighborhood best
#define VMAX 1.0                       // Maximum velocity particle can attain
#define MININIT 0.0                   // Lower bound on initialization value
#define MAXINIT 10.0                    // Upper bound on initialization value
#define ITS 25                          // Number of iterations to run

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_ITS 500                     // Number of fitness steps to run during evolution

#define FINALRUNS 10
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8
#define N_RUNS 1

#define PI 3.1415926535897932384626433832795
#define MIGRATION_BIAS  0.4

static WbNodeRef robs[MAX_ROB];
WbDeviceTag emitter;
WbDeviceTag phy_emitter;
WbDeviceTag receiver;
WbFieldRef ds[MAX_ROB][NB_SENSOR];
float ds_values[MAX_ROB][NB_SENSOR];

const double *loc[MAX_ROB];
const double *rot[MAX_ROB];
double new_loc[MAX_ROB][3];
double new_rot[MAX_ROB][4];
float old_cm[2] = {0,0};          // Previous center of mass
float curr_cm[2] = {0,0};         // Current center of mass
int t_old_cm;                     // Timestamp of the computation of old_cm
int t_curr_cm;                    // Timestamp of the computation of curr_cm
float migr_x;
float migr_z;

float orient_migr;                // Migration orientation

int random_pos_state;

double* pso(int,int,double,double,double,double,double,int,int,int);
void fitness(double[][DATASIZE],double[],int[][SWARMSIZE]);                       // Fitness function for particle evolution
double rnd(void);

void calc_fitness(double[][DATASIZE],double[],int,int);
void random_pos(int);
void nRandom(int[][SWARMSIZE],int);
void nClosest(int[][SWARMSIZE],int);
void fixedRadius(int[][SWARMSIZE],double);
double robdist(int i, int j);
double rob_orientation(int i);

/* RESET - Get device handles and starting locations */
int reset(int argc, char *args[]){
    // Device variables
    char rob[] = "epuck0";
    char rob2[] = "epuck10";

    int i;
    //For robot numbers < 10
    for (i=0;i<10 && i<MAX_ROB;i++) {
        robs[i] = wb_supervisor_node_get_from_def(rob);
        loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        new_loc[i][0] = loc[i][0]; new_loc[i][1] = loc[i][1]; new_loc[i][2] = loc[i][2];
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];
        rob[5]++;
        /*
        char s[4] = "ps0";
        for(j=0; i<NB_SENSOR;i++) {
            ds[i][j]=wb_supervisor_node_get_field(robs[i], s);
            ds_values[i][j] = 0;
            s[2]++;
        }
        */
    }
    //For robot numbers < 20
    for (i=10;i<20 && i<MAX_ROB;i++) {
        robs[i] = wb_supervisor_node_get_from_def(rob2);
        loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        new_loc[i][0] = loc[i][0]; new_loc[i][1] = loc[i][1]; new_loc[i][2] = loc[i][2];
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];
        rob2[4]++;
        /*
        char s[4] = "ps0";
        for(j=0; i<NB_SENSOR;i++) {
            ds[i][j]=wb_supervisor_node_get_field(robs[i], s);
            ds_values[i][j] = 0;
            s[2]++;
        }
        */
    }

    receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");

    char phy_emitter_name[] = "physics_emitter";
    phy_emitter = wb_robot_get_device(phy_emitter_name);
    if (phy_emitter==0) printf("physics emitter not found\n");

    if (argc == 3) { // Get parameters
        migr_x = atof(args[1]);
        migr_z = atof(args[2]);
        //migration goal point comes from the controller arguments. It is defined in the world-file, under "controllerArgs" of the supervisor.
        printf("Migratory goal : (%f, %f)\n", migr_x, migr_z);
    } else {
        printf("Missing argument\n");
        return 1;
    }
    orient_migr = -atan2f(migr_z,migr_x);
    return 0;
}

/* MAIN - Distribute and test conctrollers */
int main(int argc, char *args[]){
    double *weights;                         // Optimized result
    int i,j,k;				     // Counter variables

    /* Initialisation */
    wb_robot_init();
    printf("Particle Swarm Optimization Super Controller\n");
    if(reset(argc, args)){
        return 1;
    }
    wb_receiver_enable(receiver,TIME_STEP);
    wb_receiver_set_channel(receiver, 3);
    wb_emitter_set_channel(emitter, 3);
    wb_robot_step(256);

    double fit, w[ROBOTS][DATASIZE], f[ROBOTS];

    // Optimize controllers
    //endfit = 0.0;

    // Do N_RUNS runs and send the best controller found to the robot
    for (j=0;j<N_RUNS;j++) {
        // Get result of optimization
        weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE,ROBOTS);

        // Set robot weights to optimization results
        fit = 0.0;
        for (i=0;i<ROBOTS;i++) {
            for (k=0;k<DATASIZE;k++) {
              w[i][k] = weights[k];
              printf("BEST WEIGHTS %d: %f\n", k,weights[k]);
            }
        }

        // Run FINALRUN tests and calculate average
        printf("Running final runs\n");
        for (i=0;i<FINALRUNS;i+=ROBOTS) {
            calc_fitness(w,f,FIT_ITS,ROBOTS);
            for (k=0;k<ROBOTS && i+k<FINALRUNS;k++) {
                //fitvals[i+k] = f[k];
                fit += f[k];
            }
        }

        fit /= FINALRUNS;  // average over the 10 runs

        printf("Average Performance: %.3f\n",fit);
    }

    /* Wait forever */
    while (1){
        calc_fitness(w,f,FIT_ITS,ROBOTS);
    }

    return 0;
}

// Makes sure no robots are overlapping
char valid_locs(int rob_id) {
    int i;
    for (i = 0; i < MAX_ROB; i++) {
        if (rob_id == i) continue;
        if ((pow(new_loc[i][0]-new_loc[rob_id][0],2) +
                pow(new_loc[i][2]-new_loc[rob_id][2],2)) < (2*ROB_RAD+0.0002)*(2*ROB_RAD+0.0002)) {
        return 0;
      }
    }
    return 1;
}

// Randomly position specified robot
void random_pos(int rob_id) {
    //printf("Setting random position for %d\n",rob_id);
    if(random_pos_state%3 == 0) {
      do {
          new_rot[rob_id][0] = 0.0;
          new_rot[rob_id][1] = 1.0;
          new_rot[rob_id][2] = 0.0;
          new_rot[rob_id][3] = -1*PI + rnd()*2*PI;
          new_loc[rob_id][0] = -0.350 + rob_id*0.150 ;
          new_loc[rob_id][1] = -0.1;
          new_loc[rob_id][2] = 0.475;
          //printf("%d at %.2f, %.2f\n", rob_id, new_loc[rob_id][0], new_loc[rob_id][2]);
      } while (!valid_locs(rob_id));
    } else { if(random_pos_state%3 == 1) {
        do {
            // X between -0.350 and +0.350
            // Z between 0.450 and 0.800
            new_rot[rob_id][0] = 0.0;
            new_rot[rob_id][1] = 1.0;
            new_rot[rob_id][2] = 0.0;
            new_rot[rob_id][3] = -1*PI + rnd()*2*PI;
            new_loc[rob_id][0] = -0.350 + rnd()*0.700;
            new_loc[rob_id][1] = -0.1;
            new_loc[rob_id][2] = 0.450 + rnd()*0.350;
            //printf("%d at %.2f, %.2f\n", rob_id, new_loc[rob_id][0], new_loc[rob_id][2]);
        } while (!valid_locs(rob_id));
      }
      else {
        do {
          new_rot[rob_id][0] = 0.0;
          new_rot[rob_id][1] = 1.0;
          new_rot[rob_id][2] = 0;
          new_rot[rob_id][3] = -1*PI + rnd()*2*PI;
          new_loc[rob_id][0] = 0;
          new_loc[rob_id][1] = -0.1;
          new_loc[rob_id][2] = 0.400 + rob_id*0.100;
            //printf("%d at %.2f, %.2f\n", rob_id, new_loc[rob_id][0], new_loc[rob_id][2]);
        } while (!valid_locs(rob_id));
      }
    }


    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[rob_id],"translation"), new_loc[rob_id]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[rob_id],"rotation"), new_rot[rob_id]);
}

// Distribute fitness functions among robots
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// TODO : complete the function below to calculate the fitness
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs) {
    double buffer[255];
    int i,j;
    random_pos_state++;
    for(j=0; j<DATASIZE; j++) {
      //printf("Weight %d: %f\n", j, weights[0][j]);
    }

	  // Compute reference fitness values
	  float fit_orient;			// Performance metric for orientation
    float fit_cohesion;			// Performance metric for fit_cohesion
    float fit_velocity;			// Performance metric for fit_velocity
    float fit_sep;
    float fit_instant;          // Instant Performance of the flock
    float fit_sum = 0;          // Sum of the Instant Performances of the flock
    int t=0;                    // Time in ms.
    int step_count = 0;         // Number of timesteps

    for(i=0; i<MAX_ROB; i++) {
      for(j=0; j<3; j++) {
          new_loc[i][j] = 0;
      }
    }
    /* Send data to robots */
    for(i=0; i<MAX_ROB; i++) {
      random_pos(i);
      for (j=0; j<DATASIZE; j++) {
        buffer[j] = weights[0][j];
      }
      buffer[DATASIZE] = its;
      loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
      rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
    }
    wb_emitter_send(emitter,(void *)buffer,(DATASIZE+1)*sizeof(double));

    //Initialize center of mass.
    float init_cm[2];
    wb_supervisor_simulation_reset_physics();
    compute_center_of_mass(init_cm);
	 /* Get the positions while waiting for response from robots */
    while (wb_receiver_get_queue_length(receiver) == 0){
        wb_robot_step(TIME_STEP);
        for(i=0; i<MAX_ROB; i++) {
          loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
          rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        }

        compute_center_of_mass(curr_cm);
        t_curr_cm = t;
        fit_sep = 0;
        for(i=0; i<MAX_ROB; i++) {
          float dist_to_cm = sqrt(pow(loc[i][0] - curr_cm[0], 2) + pow(loc[i][2] - curr_cm[1], 2));

          if( dist_to_cm < DESIRED_CM) {
            fit_sep += DESIRED_CM - dist_to_cm;
          }
        }

        fit_sep = 1 - fit_sep/(DESIRED_CM*MAX_ROB);

        //Compute and normalize fitness values
        compute_fitness(&fit_orient, &fit_cohesion, &fit_velocity);
        fit_instant = 0.3*fit_orient + 0.4*fit_cohesion + 0.3*fit_sep;
        fit_sum += fit_instant;

        step_count++;
        t += TIME_STEP;
    }
    /* Receive messages from robots */
    for (i=0;i<MAX_ROB;i++) {
        wb_receiver_next_packet(receiver);
    }

    float end_cm[2];
    compute_center_of_mass(end_cm);
    double dist_y_cm = sqrt(pow(init_cm[1] - end_cm[1], 2));
    fit_sum = 0.9*fit_sum/step_count + 0.1*dist_y_cm/MAX_DIST;

  for (i = 0; i<ROBOTS; i++) {
    fit[i] = fit_sum;
  }
  printf("Particle fitness: %.3f\n", fit[0]);

}

/* Optimization fitness function , used in pso.c */
/************************************************************************************/
/* Use the NEIHBORHOOD definition at the top of this file to                        */
/* change the neighborhood type for the PSO. The possible values are:               */
/* STANDARD    : Local neighborhood with 2*NB (defined above) nearest neighbors     */
/*               NEIGHBORHOOD is set to STANDARD by default                         */
/* RAND_NB     : 2*NB random neighbors                                              */
/* NCLOSE_NB   : 2*NB closest neighbors                                             */
/* FIXEDRAD_NB : All robots within a defined radius are neighbors                   */
/************************************************************************************/
void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE]) {
    calc_fitness(weights,fit,FIT_ITS,ROBOTS);

#if NEIGHBORHOOD == RAND_NB
    nRandom(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == NCLOSE_NB
    nClosest(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == FIXEDRAD_NB
    fixedRadius(neighbors,RADIUS);
#endif
}

/* Get distance between robots */
double robdist(int i, int j) {
    return sqrt(pow(loc[i][0]-loc[j][0],2) + pow(loc[i][2]-loc[j][2],2));
}

/* Choose n random neighbors */
void nRandom(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

    int i,j;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Clear old neighbors */
        for (j = 0; j < ROBOTS; j++)
        	neighbors[i][j] = 0;

        /* Set new neighbors randomly */
        for (j = 0; j < numNB; j++)
        	neighbors[i][(int)(SWARMSIZE*rnd())] = 1;

    }
}

/* Choose the n closest robots */
void nClosest(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

    int r[numNB];
    int tempRob;
    double dist[numNB];
    double tempDist;
    int i,j,k;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Clear neighbors */
        for (j = 0; j < numNB; j++)
        dist[j] = ARENA_SIZE;

        /* Find closest robots */
        for (j = 0; j < ROBOTS; j++) {

            /* Don't use self */
            if (i == j) continue;

            /* Check if smaller distance */
            if (dist[numNB-1] > robdist(i,j)) {
                dist[numNB-1] = robdist(i,j);
                r[numNB-1] = j;

                /* Move new distance to proper place */
                for (k = numNB-1; k > 0 && dist[k-1] > dist[k]; k--) {

                    tempDist = dist[k];
                    dist[k] = dist[k-1];
                    dist[k-1] = tempDist;
                    tempRob = r[k];
                    r[k] = r[k-1];
                    r[k-1] = tempRob;

                }
            }

        }

        /* Update neighbor table */
        for (j = 0; j < ROBOTS; j++)
        neighbors[i][j] = 0;
        for (j = 0; j < numNB; j++)
        neighbors[i][r[j]] = 1;

    }

}

/* Choose all robots within some range */
void fixedRadius(int neighbors[SWARMSIZE][SWARMSIZE], double radius) {

    int i,j;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Find robots within range */
        for (j = 0; j < ROBOTS; j++) {

            if (i == j) continue;

            if (robdist(i,j) < radius) neighbors[i][j] = 1;
            else neighbors[i][j] = 0;

        }

    }

}

void step_rob() {
    wb_robot_step(TIME_STEP);
}

/*
 * Compute center of mass.
 */
void compute_center_of_mass(float cm[2]) {
    int i;
    float cm_x = 0; // x coordinate of center of mass
    float cm_z = 0; // z coordinate of center of mass

    for (i=0;i<MAX_ROB;i++) {
        cm_x += loc[i][0];
        cm_z += loc[i][2];
    }
    cm_x /= MAX_ROB;
    cm_z /= MAX_ROB;

    cm[0] = cm_x;
    cm[1] = cm_z;
}

/*
 * Compute orientation performance metric.
 */
void compute_orientation_fitness(float* fit_o) {
    int i;
    float cos_sum = 0, sin_sum = 0;

    for (i=0;i<MAX_ROB;i++) {
        cos_sum += cosf(rot[i][3]);
        sin_sum += sinf(rot[i][3]);
    }

    *fit_o = sqrtf(cos_sum*cos_sum + sin_sum*sin_sum) / MAX_ROB;
}

/*
 * Compute cohesion performance metric.
 */
void compute_cohesion_fitness(float* fit_c) {
    int i;

    float sum_dist_to_cm = 0; // distances to center of mass

    for (i=0;i<MAX_ROB;i++) {
        sum_dist_to_cm += sqrtf(powf(curr_cm[0]-loc[i][0],2)+powf(curr_cm[1]-loc[i][2],2));
    }

    *fit_c = 1 / (1 + sum_dist_to_cm / MAX_ROB);
}


/*
 * Compute velocity performance metric.
 */
void compute_velocity_fitness(float* fit_v) {
    float cm_velocity_x, cm_velocity_z;

    cm_velocity_x = (curr_cm[0]-old_cm[0]) / (t_curr_cm-t_old_cm) * 1000.;
    cm_velocity_z = (curr_cm[1]-old_cm[1]) / (t_curr_cm-t_old_cm) * 1000.;

    float proj = (migr_x*cm_velocity_x + migr_z*cm_velocity_z) / sqrtf(migr_x*migr_x+migr_z*migr_z);

    float max = proj > 0 ? proj : 0;

    *fit_v = 1 / V_MAX * max;
}

/*
 * Compute performance metric.
 */
void compute_fitness(float* fit_o, float* fit_c, float* fit_v) {
    *fit_c = 0; *fit_o = 0; *fit_v = 0;
    compute_orientation_fitness(fit_o);
    compute_cohesion_fitness(fit_c);
    compute_velocity_fitness(fit_v);
}

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	5 		// Number of robots in flock
#define TIME_STEP	64		// [ms] Length of time step

#define V_MAX       0.1287              // [m/s] e-puck maximum speed
#define NB_FILES    6                      // Nb of output files

WbNodeRef robs[FLOCK_SIZE];		        // Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	    // Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields

float loc[FLOCK_SIZE][3];		  // Location of everybody in the flock


int offset;				          // Offset of robots number
float migr_x, migr_z;			  // Migration vector
float orient_migr; 			      // Migration orientation
int t;
int step_count = 1;

float old_cm[2] = {0,0};          // Previous center of mass
float curr_cm[2] = {0,0};         // Current center of mass
int t_old_cm;                     // Timestamp of the computation of old_cm
int t_curr_cm;                    // Timestamp of the computation of curr_cm

FILE *files[NB_FILES];                   // Output files
// 0 -> instant orientation perf
// 1 -> instant cohesion perf
// 2 -> instant velocity perf
// 3 -> instant perf
// 4 -> overall perf
// 5 -> clock tick

/*
 * Initialize controller arguments
 */
int init_args(int argc, char *args[]) {
    if (argc == 4) { // Get parameters
        offset = atoi(args[1]);
        migr_x = atof(args[2]);
        migr_z = atof(args[3]);
        //migration goal point comes from the controller arguments. It is defined in the world-file, under "controllerArgs" of the supervisor.
        printf("Migratory instinct : (%f, %f)\n", migr_x, migr_z);
    } else {
        printf("Missing argument\n");
        return 1;
    }

    return 0;
}

void initFile(int index_file, char* field_name){
    char filename[512];
    sprintf(filename, "../../../matlab/output_%s_%d.m", field_name, offset);
    files[index_file] = fopen(filename,"w");
    fprintf(files[index_file], "%s%d = [", field_name, offset);
}

/*
 * Initialize flock position and devices
 */
void reset(void) {
	wb_robot_init();

	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i+offset*FLOCK_SIZE);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}

    initFile(0, "orientation");
    initFile(1, "cohesion");
    initFile(2, "velocity");
    initFile(3, "instant");
    initFile(4, "overall");
    initFile(5, "clock");
}

/*
 * Compute center of mass.
 */
void compute_center_of_mass(float cm[2]) {
    int i;
    float cm_x = 0; // x coordinate of center of mass
    float cm_z = 0; // z coordinate of center of mass

    for (i=0;i<FLOCK_SIZE;i++) {
        cm_x += loc[i][0];
        cm_z += loc[i][1];
    }
    cm_x /= FLOCK_SIZE;
    cm_z /= FLOCK_SIZE;

    cm[0] = cm_x;
    cm[1] = cm_z;
}

/*
 * Compute orientation performance metric.
 */
void compute_orientation_fitness(float* fit_o) {
    int i;
    float cos_sum = 0, sin_sum = 0;

    for (i=0;i<FLOCK_SIZE;i++) {
        cos_sum += cosf(loc[i][2]);
        sin_sum += sinf(loc[i][2]);
    }

    *fit_o = sqrtf(cos_sum*cos_sum + sin_sum*sin_sum) / FLOCK_SIZE;
}

/*
 * Compute cohesion performance metric.
 */
void compute_cohesion_fitness(float* fit_c) {
    int i;

    float sum_dist_to_cm = 0; // distances to center of mass

    for (i=0;i<FLOCK_SIZE;i++) {
        sum_dist_to_cm += sqrtf(powf(curr_cm[0]-loc[i][0],2)+powf(curr_cm[1]-loc[i][1],2));
    }

    *fit_c = 1 / (1 + sum_dist_to_cm / FLOCK_SIZE);
}

/*
 * Compute velocity performance metric.
 */
void compute_velocity_fitness(float* fit_v) {
    float cm_velocity_x, cm_velocity_z;

    cm_velocity_x = (curr_cm[0]-old_cm[0]) / (t_curr_cm-t_old_cm) * 1000;
    cm_velocity_z = (curr_cm[1]-old_cm[1]) / (t_curr_cm-t_old_cm) * 1000;

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

/*
 * Main function.
 */
 
int main(int argc, char *args[]) {
	int i;			// Index

    if (init_args(argc, args)) {
        return 1;
    }
	reset();

	// Compute reference fitness values
	float fit_orient;			// Performance metric for orientation
    float fit_cohesion;			// Performance metric for fit_cohesion
    float fit_velocity;			// Performance metric for fit_velocity

    float fit_instant;          // Instant Performance of the flock
    float fit_sum = 0;          // Sum of the Instant Performances of the flock

    bool first_calculation = true; // first time we compute the performances
		
	while (wb_robot_step(TIME_STEP) != -1) {
		
		if (t % 10 == 0) {
            // Get data
            for (i=0;i<FLOCK_SIZE;i++) {
                loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
                loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[2]; // Z
                loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
            }
		    if (first_calculation) {
                compute_center_of_mass(old_cm);
                t_old_cm = t;
                first_calculation = false;
		    } else {
                compute_center_of_mass(curr_cm);
                t_curr_cm = t;
                //Compute and normalize fitness values
                compute_fitness(&fit_orient, &fit_cohesion, &fit_velocity);

                fit_instant = fit_orient * fit_cohesion * fit_velocity;

                fit_sum += fit_instant;

				fprintf(files[0],"%f\n", fit_orient);
				fprintf(files[1],"%f\n", fit_cohesion);
				fprintf(files[2],"%f\n", fit_velocity);
				fprintf(files[3],"%f\n", fit_instant);
				fprintf(files[4],"%f\n", fit_sum/step_count);
				fprintf(files[5],"%d\n", t);

                //printf("time:%d, Instant Performance: %f\n", t, fit_instant);
                //printf("time:%d, Overall Performance: %f\n", t, fit_sum/step_count);

                old_cm[0] = curr_cm[0];
                old_cm[1] = curr_cm[1];
                t_old_cm = t_curr_cm;
                step_count++;
		    }
		}
		
		t += TIME_STEP;
	}

	int f_idx = 0;
	for(f_idx = 0; f_idx < NB_FILES; f_idx++){
        fprintf(files[f_idx],"];\n");
        fclose(files[f_idx]);
	}
	
	wb_robot_cleanup();
	return 0;

}

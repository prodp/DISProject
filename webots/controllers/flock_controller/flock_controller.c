#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>


#define NB_SENSORS      8       // Number of distance sensors
#define MIN_SENS        100     // Minimum sensibility value
#define MAX_SENS        500    // Maximum sensibility value

#define MAX_SPEED       1000    // Maximum speed
#define MAX_SPEED_WEB   6.28    // Maximum speed webots
#define MAX_RADIO_RANGE 1.5     // Maximum radio range

#define FLOCK_SIZE      5       // Size of flock

#define AXLE_LENGTH     0.052   // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS 0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS    0.0205  // Wheel radius (meters)

#define O_NO_DETECTION  0       // Zero detection value
#define O_MIN           1       // Minimal detection value
#define O_MAX           7       // Maximal detection value

#define C               (O_MAX-O_NO_DETECTION)*(O_MAX-O_NO_DETECTION)    // Scaling constance
#define GAMMA           1.0       // Activator for modulation of the forward velocity
#define KP              0.5   // Proportional gain for angular velocity
#define ALPHA           2.9       // Proportional constant for heading
#define BETA            1.8      // Weight of proximal control
#define DELTA           2.4     // Proportional constant for cohesion
#define ZETA            0.5     // Gain for cohesion non-linearity
#define IOTA            0.9     // Gain for proximal non-linearity
#define MIGRATION_BIAS  0.4     // Migration bias
#define DESIRED_CM      0.15 // Desired distance to center of mass [m]

#define EPSILON         1e-6

#define TIME_STEP       64      // [ms] Length of time step
#define TIMEOUT         1000    // [ms] Timeout of IRSS communication
#define TIMEOUT_STEPS   (long)TIMEOUT/TIME_STEP   // Number of steps before a timeout

#define ABS(x) ((x>=0)?(x):-(x))

/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

float e_puck_sensor_angles[NB_SENSORS] = {-0.2967,-0.8727, -1.5708, -2.6180, 2.6180, 1.5708, 0.8727, 0.2967};

WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

float ds_values[NB_SENSORS];

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID
char* robot_name;
int flock;
long tick;

float my_position[3];       // X, Z, Theta of the current robot

float relative_pos[FLOCK_SIZE][3];      // relative X, Z, Theta of all robots

long last_heard_of[FLOCK_SIZE];

int initialized[FLOCK_SIZE];            // != 0 if initial positions have been received


//================================================================================
// Reset e-puck before each simulation
static void reset(){

    wb_robot_init();

    //Initialize motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    //Initialize proximity sensors
    int i;
    char s[4]="ps0";
    for(i=0; i<NB_SENSORS;i++) {
        ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
        wb_distance_sensor_enable(ds[i],TIME_STEP);
        ds_values[i] = 0;
        s[2]++;				// increases the device number
    }

    //Initialize robot id and flock id
    robot_name =(char*) wb_robot_get_name();
    sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
    flock = 0;

    flock = robot_id_u / FLOCK_SIZE;
    robot_id = robot_id_u%FLOCK_SIZE; // normalize between 0 and FLOCK_SIZE-1

    //Initialize Radio
    receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");
    wb_receiver_enable(receiver,TIME_STEP);

    //Initialize position.
    for (int j=0; j<=2; j++) {
        my_position[j] = 0;
    }

    for (int j=0; j<FLOCK_SIZE; j++) {
        for(i = 0; i < 2; i++){
            relative_pos[j][i] = 0;
        }
        initialized[j] = 0;
        last_heard_of[j] = 0;
    }
    tick = 0;

    wb_robot_step(TIME_STEP);

}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit_sym(int *number, int limit){
    if(*number > limit){
        *number = limit;
    }
    if(*number < -limit){
        *number = -limit;
    }
}

void limit_and_rescale_sym(int *msl, int *msr, int limit){
    if(*msl > limit || *msl < -limit){
        float rescale = ABS((float)limit/(*msl));
        *msl *= rescale;
        *msr *= rescale;
    }
    if(*msr > limit || *msr < -limit){
        float rescale = ABS((float)limit/(*msr));
        *msl *= rescale;
        *msr *= rescale;
    }
}

/*
 * Keep given int number within interval {limit1, limit2}
 */
void limit(int *number, int limit1, int limit2){
    if(*number > limit2){
        *number = limit2;
    }
    if(*number < limit1){
        *number = limit1;
    }
}

/*
 * Compute sign of input number
 */
int sign(float x) {
  return (x < 0) ? -1 : (x > 0);
}

/*
 * Compute the square norm of an input vector of given size
 */
float sq_norm(float *vect, int size){
    float sqnorm = 0.f;
    for(int i = 0; i < size; ++i){
        sqnorm += vect[i]*vect[i];
    }
    return sqnorm;
}

/*
 * Rescale a value from a range to an other
 */
float warp(float val, float old_a, float old_b, float new_a, float new_b){

    return (val-old_a)/(old_b-old_a)*(new_b-new_a)+new_a;
}

/*
 * Compute the detection strength
 */
float compute_O_k(float detection){
    if(detection < MIN_SENS){
        return O_NO_DETECTION;
    }
    if(detection > MAX_SENS){
        return O_MAX;
    }
    return warp(detection, MIN_SENS, MAX_SENS, O_MIN, O_MAX);
}

/*
 * Compute the speed of the robot for next time step
 */
void compute_speed(int *msl, int *msr){
    float a[2] = {0,0};
    float h[2] = {0,0};
    float p[2] = {0,0};
    float c[2] = {0,0};

    // Heading component
    for(int i = 0; i<FLOCK_SIZE;++i){
        if(i == robot_id){
            continue;
        }
        h[0] += cosf(relative_pos[i][2]);
        h[1] += sinf(relative_pos[i][2]);
    }
    h[0] += 1; // own heading


    float h_norm = sqrtf(sq_norm(h,2));
    h[0] /= h_norm;
    h[1] /= h_norm;

    h[0] = (1-MIGRATION_BIAS)*h[0]+MIGRATION_BIAS*cosf(0-my_position[2]);
    h[1] = (1-MIGRATION_BIAS)*h[1]+MIGRATION_BIAS*sinf(0-my_position[2]);

    // Proximal component
    float max_sens_value = 0;
    for(int i = 0; i < NB_SENSORS; ++i){
        float value = wb_distance_sensor_get_value(ds[i]); //Read sensor values
        if(value > max_sens_value){
            max_sens_value = value;
        }
        float O_k = compute_O_k(value);
        float fk = -O_k*O_k/C;

        p[0] += fk*cosf(e_puck_sensor_angles[i]);
        p[1] += fk*sinf(e_puck_sensor_angles[i]);
    }
    p[0] /= NB_SENSORS;
    p[1] /= NB_SENSORS;
    float norm_p = sqrtf(sq_norm(p, 2));
    if(norm_p > 0) {
        p[0] /= norm_p;
        p[1] /= norm_p;
    }else{
        p[0] = 0;
        p[1] = 0;
    }

    // cohesion vector
    float distance_to_cm = 0;
    float denom = 1;
    for(int i = 0; i<FLOCK_SIZE;++i){
        if(i == robot_id || (relative_pos[i][0] == 0 && relative_pos[1] == 0)){
            continue;
        }
        c[0] += relative_pos[i][0];
        c[1] += relative_pos[i][1];
        denom++;
    }
    c[0] /= denom;
    c[1] /= denom;
    distance_to_cm = sqrtf(sq_norm(c, 2));
    if(distance_to_cm > 0){
        c[0] /= distance_to_cm;
        c[1] /= distance_to_cm;
    }else {
        c[0] = 0;
        c[1] = 0;
    }
    float c_coeff = distance_to_cm-DESIRED_CM > 0 ? pow(DELTA,ZETA+ distance_to_cm-DESIRED_CM) : 0;

    // heading vector
    a[0] = ALPHA*h[0]+pow(BETA,IOTA+ max_sens_value/MAX_SENS)*p[0]+c_coeff*c[0];
    a[1] = ALPHA*h[1]+pow(BETA,IOTA+ max_sens_value/MAX_SENS)*p[1]+c_coeff*c[1];
    
    float a_norm = sqrtf(sq_norm(a,2));
    if(a_norm > EPSILON){
        a[0] /= a_norm;
        a[1] /= a_norm;
    }else{
        a[0] = 0;
        a[1] = 1;
    }

    // speeds
    float phase_a = atan2(a[1],a[0]);

    float dot_c_h =0;
    if(distance_to_cm - DESIRED_CM > 0) {
      dot_c_h = (c[0]*h[0]+c[1]*h[1]);
     }

    float dot = a[0];
    float u = 0; // Speed in m.s⁻1
    if(dot > 0){
        if(GAMMA != 0){
            if(dot_c_h<0 && p[0] >= 0)
              u = dot*2*M_PI*WHEEL_RADIUS*sqrt(1-ABS(dot_c_h));
            else
              u = dot*2*M_PI*WHEEL_RADIUS;
        }
        else{
            u = 2*M_PI*WHEEL_RADIUS;
        }
    } else if(dot_c_h<0) {
        u = ABS(dot)*2*M_PI*WHEEL_RADIUS*sqrt(1-ABS(dot_c_h));
    }
    float w = (phase_a)*KP*8*WHEEL_RADIUS/AXLE_LENGTH;
    
    *msl = (u - w*AXLE_LENGTH/2)*MAX_SPEED/(2*M_PI*WHEEL_RADIUS);
    *msr = (u + w*AXLE_LENGTH/2)*MAX_SPEED/(2*M_PI*WHEEL_RADIUS);

    limit_and_rescale_sym(msl, msr,MAX_SPEED-1);
}

/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) {
    float theta = my_position[2];

    // Compute deltas of the robot
    float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000.;
    float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000.;
    float du = (dr + dl)/2.0;
    float dtheta = (dr - dl)/AXLE_LENGTH;

    // Compute deltas in the environment
    float dx = -du * sinf(theta);
    float dz = -du * cosf(theta);

    // Update position
    my_position[0] += dx;
    my_position[1] += dz;
    my_position[2] += dtheta;

    // Keep orientation within [-pi,pi]
    if (my_position[2] > M_PI) my_position[2] -= 2.0*M_PI;
    if (my_position[2] < -M_PI) my_position[2] += 2.0*M_PI;
}

//================================================================================
// Receive Range and Bearing messages from the queue (here radio)
void receive_packages() {
    const double *message_direction;
    double message_rssi; // Received Signal Strength indicator
    double bearing;
    double range;
    const char *inbuffer;
    int other_flock;
    int other_id;
    float other_theta;

    while (wb_receiver_get_queue_length(receiver) > 0) {
        inbuffer = (char*) wb_receiver_get_data(receiver);
        other_id = (int)(inbuffer[0]-'0');
        other_flock = (int)(inbuffer[1] - '0');
        char buf[20];
        sprintf(buf, "%s", inbuffer+2);
        other_theta = atof(buf);

        message_rssi = wb_receiver_get_signal_strength(receiver);
        range = sqrt(1/message_rssi);
        if((other_flock == flock) && (other_id != robot_id) && (range < MAX_RADIO_RANGE)) {
            message_direction = wb_receiver_get_emitter_direction(receiver);
            //Get relative position and angle of message
            bearing = atan2(message_direction[0],message_direction[2]);
            float other_pos[2] = {-1*range*cos(bearing), -1*range*sin(bearing)};
            //Update position
            for(int i=0; i<2; i++) {
                relative_pos[other_id][i] = other_pos[i];
            }
            last_heard_of[other_id] = tick;
            relative_pos[other_id][2] = other_theta-my_position[2];
            if (relative_pos[other_id][2] > M_PI) relative_pos[other_id][2] -= 2.0*M_PI;
            if (relative_pos[other_id][2] < -M_PI) relative_pos[other_id][2] += 2.0*M_PI;

            last_heard_of[other_id] = tick;
            initialized[other_id] = 1;
        }
        wb_receiver_next_packet(receiver);
    }
}

//================================================================================
// Send messages to other e-pucks
void emit_packages() {
    char out[255];
    int j = sprintf(out,"%.1d",robot_id);  // in the ping message we send the name of the robot.
    j += sprintf(out+j, "%.1d", flock);
    j += sprintf(out+j, "%.5f", my_position[2]);

    wb_emitter_send(emitter,out,strlen(out));
}

// the main function
int main(){

    int msl, msr;
    float msl_w, msr_w;

    reset();

    msl = 0;
    msr = 0;

    while(1){
        update_self_motion(msl,msr);

        emit_packages();

        receive_packages();

        // if robots are out of range, put their heading to the migration direction
        for(int i = 0; i<FLOCK_SIZE;++i){
            if((tick-last_heard_of[i])>TIMEOUT_STEPS || !initialized[i]){
                relative_pos[i][2] = 0-my_position[2];
                relative_pos[i][0] = 0;
                relative_pos[i][1] = 0;
            }
        }

        compute_speed(&msl,&msr);
        // Set speed
        msl_w = msl*SPEED_UNIT_RADS;
        msr_w = msr*SPEED_UNIT_RADS;
        wb_motor_set_velocity(left_motor, msl_w);
        wb_motor_set_velocity(right_motor, msr_w);

        // Continue one step
        wb_robot_step(TIME_STEP);
        tick += 1;
    }

    wb_robot_cleanup();
    return 0;
}

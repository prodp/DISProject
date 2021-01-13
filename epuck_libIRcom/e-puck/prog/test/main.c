#include <ircom/e_ad_conv.h>
#include <epfl/e_init_port.h>
#include <epfl/e_epuck_ports.h>
#include <epfl/e_uart_char.h>
#include <epfl/e_led.h>

#include <epfl/e_led.h>
#include <epfl/e_motors.h>
#include <epfl/e_agenda.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ircom/ircom.h>
#include <btcom/btcom.h>
#include <math.h>

#define M_PI        3.14159265358979323846264338327950288   /* pi             */
#define M_PI_2      1.57079632679489661923132169163975144   /* pi/2           */

#define AXLE_LENGTH  5.2 // distance between wheels
#define WHEEL_RADIUS 2.05 // diameter of wheels

#define SPEED_UNIT_RADS 0.00628 // Conversion factor from speed unit to radian per second

#define MIN_SENS        100     // Minimum sensibility value
#define MAX_SENS        500     // Minimum sensibility value

#define MAX_SPEED       200    // Maximum speed

#define FLOCK_SIZE      3       // Size of flock

#define O_NO_DETECTION  0       // Zero detection value
#define O_MIN           1       // Minimal detection value
#define O_MAX           7       // Maximal detection value

#define C               (O_MAX-O_NO_DETECTION)*(O_MAX-O_NO_DETECTION)    // Scaling constance
#define GAMMA           1.0       // Activator for modulation of the forward velocity
#define KP              0.5   // Proportional gain for angular velocity
#define ALPHA           2.9       // Proportional constant for heading
#define BETA            1.8      // Weight of proximal control
#define DELTA           1.5     // Proportional constant for cohesion
#define ZETA            0.8     // Gain for cohesion non-linearity
#define IOTA            0.7     // Gain for proximal non-linearity
#define MIGRATION_BIAS  0.6
#define DESIRED_CM      7.5 // Desired distance to center of mass [cm]

#define EPSILON         1e-6

#define TIME_STEP        100
#define TIME_STEP_WAIT   1000
#define TIME_RECEIVE     TIME_STEP_WAIT-TIME_STEP
#define TIMEOUT         10000

#define ABS(x) ((x>=0)?(x):-(x))
#define MAX(x,y) ((x >= y) ? (x) : (y))

int robot_id;
int flock;
int tick;

float my_position[3];       // X, Z, Theta of the current robot
float relative_pos[FLOCK_SIZE][3];      // relative X, Z, Theta of all robots

unsigned long last_heard_of[FLOCK_SIZE];

int initialized[FLOCK_SIZE];            // != 0 if initial positions have been received

unsigned long lastClock;
volatile unsigned long clock;

float sensorDir[NB_IR_SENSORS] = {-0.2967,-0.8727, -1.5708, -2.6180, 2.6180, 1.5708, 0.8727, 0.2967};

void init();

int round(double nb);

void print_robot_msg(int robot_id, int flock, double theta);

void emit();

void receive();

void limit_sym(int *number, int limit);

void limit_and_rescale_sym(int *msl, int *msr, int limit);

void limit(int *number, int limit1, int limit2);

int sign(float x);

float sq_norm(float *vect, int size);

float warp(float val, float old_a, float old_b, float new_a, float new_b);

float compute_O_k(float detection);

void update_speeds(int *msl, int *msr);

void update_self_motion(int msl, int msr);

int round(double nb){
    return (nb + 0.5);
}

int getselector()
{
    return SELECTOR0 + 2*SELECTOR1 + 4*SELECTOR2 + 8*SELECTOR3;
}

void wait(unsigned long n){
    while(ircomData.time - lastClock < TIME_STEP_WAIT){};
}

void init(){
    // init robot
    e_init_port();
    e_init_ad_scan();
    e_init_uart1();
    e_led_clear();
    e_init_motors();
    e_start_agendas_processing();

    robot_id = getselector();
    flock = robot_id/FLOCK_SIZE;
    clock = ircomData.time;
    tick = 0;

    int i;
    for(i = 0; i < 2; i++){
        my_position[i] = 0;
    }
    int j;
    for(j = 0; j < FLOCK_SIZE; j++){
        for(i = 0; i < 2; i++){
            relative_pos[j][i] = 0;
        }
        last_heard_of[j] = 0;
        initialized[j] = 0;
    }

    // initialize ircom and start reading
    ircomStart();
    ircomEnableContinuousListening();
    ircomListen();

    char tmp[256];
    sprintf(tmp, "Initialized robot %d \n\r", robot_id);
    btcomSendString(tmp);
}

int main()
{
    init();

    char tmp[256];
    btcomWaitForCommand('c');
    btcomSendString("==== IR CALIBRATING ====\n\r");
    e_calibrate_ir();
    int s;
    for(s = 0; s < NB_IR_SENSORS; s++){
        sprintf(tmp, "%d ", e_init_value_ir[s]);
        btcomSendString(tmp);
    }
    btcomSendString("\n\r");
    btcomSendString("==== IR CALIBRATING DONE ====\n\r");
    btcomWaitForCommand('s');
    btcomSendString("==== STARTING NAVIGATION ====\n\r");


    int msl = 0, msr = 0;
    while(1){
        lastClock = ircomData.time;
        update_self_motion(msl, msr);
        emit();
        receive();
        int id;
        for(id = 0; id < FLOCK_SIZE; id++){
            if(clock - last_heard_of[id] > TIMEOUT || !initialized[id]){
                relative_pos[id][0] = 0;
                relative_pos[id][1] = 0;
                relative_pos[id][2] = 0 - my_position[2];
            }
        }
        update_speeds(&msl,&msr);

        e_set_speed_left(msl);
        e_set_speed_right(msr);

        wait(TIME_STEP_WAIT);
        tick += 1;
    }
    ircomStop();
    return 0;
}

void print_robot_msg(int robot_id, int flock, double theta){
    char tmp[256];
    int j = sprintf(tmp, "id: %d ", robot_id);
    j += sprintf(tmp+j, "fl: %d ", flock);
    j += sprintf(tmp+j, "th: %.4f ", theta);
    btcomSendString(tmp);
}

void emit(){
    //print_robot_msg(robot_id, flock, my_position[2]);

    long msg = 0;
    // id => 3 bits
    msg += robot_id;
    // theta => 5 bits, 2PI => 32 (angle [0,2PI])
    float theta = my_position[2];
    if(theta < 0){
        theta = theta+2*M_PI;
    }
    int theta_idx = round(theta*16/M_PI);
    msg += (theta_idx << 3);
    ircomSend(msg);
    while (ircomSendDone() == 0);
}

void receive(){
    do
    {
        clock = ircomData.time;
        IrcomMessage imsg;
        ircomPopMessage(&imsg);
        int id_other = -1;
        if (imsg.error == 0)
        {
            //char tmp[128];
            long val = imsg.value;
            id_other = val & 7;
            if(id_other != robot_id){
                val >>= 3;
                int theta_idx_other = val & 31;
                float theta_other = theta_idx_other*M_PI/16;
                if(theta_other > M_PI){
                    theta_other = theta_other-2*M_PI;
                }
                int flock_other = id_other/FLOCK_SIZE;
                //print_robot_msg(id_other, flock_other, theta_other);
                if (flock_other == flock){
                    int id_u = id_other - FLOCK_SIZE*flock_other;
                    // distance en cm
                    double range = (double)imsg.distance;
                    // direction [0,2PI]
                    double bearing = (double)imsg.direction;
                    if(bearing > M_PI){
                        bearing -= 2*M_PI;
                    }
                    relative_pos[id_u][0] = range*cos(bearing);
                    relative_pos[id_u][1] = range*sin(bearing);
                    relative_pos[id_u][2] = theta_other;
                    last_heard_of[id_u] = clock;

                    initialized[id_u] = 1;
                }
                /*sprintf(tmp, "dst=%f dir=%f \n\r", (double)imsg.distance, (double)imsg.direction);
                btcomSendString(tmp);*/
            }
        } else if (imsg.error == -1){
            //imsg.error == -1 -> no message available in the queue
            //btcomSendString("No message available in the queue \n");
            break;
        }
    }while(clock - lastClock < (TIME_RECEIVE));
    ircomFlushMessages();
}

void update_self_motion(int msl, int msr) {
    float theta = my_position[2];

    // Compute deltas of the robot
    float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000.;
    float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * TIME_STEP/1000.;
    float du = (dr + dl)/2.0;
    float dtheta = (dr - dl)/AXLE_LENGTH;

    // Compute deltas in the environment
    float dx = -du * sin(theta);
    float dz = -du * cos(theta);

    // Update position
    my_position[0] += dx;
    my_position[1] += dz;
    my_position[2] += dtheta;
    // Keep orientation within [-pi,pi]
    if (my_position[2] > M_PI) my_position[2] -= 2.0*M_PI;
    if (my_position[2] < -M_PI) my_position[2] += 2.0*M_PI;
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
    int i;
    for(i = 0; i < size; ++i){
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
    return warp(detection, MIN_SENS, MAX_SENS, O_NO_DETECTION, O_MAX);
}

void update_speeds(int *msl, int *msr) {
    float a[2] = {0,0}; // heading vector
    float h[2] = {0,0}; // heading alignment vector
    float p[2] = {0,0}; // proximal vector
    float c[2] = {0,0}; // cohesion vector

    int id;
    // heading alignment vector
    for(id = 0; id < FLOCK_SIZE; id++){
        if(id==robot_id) {
            continue;
        }
        h[0] += cos(relative_pos[id][2]);
        h[1] += sin(relative_pos[id][2]);
    }
    // Add own heading (1,0)
    h[0] += 1;

    float h_norm = sqrtf(sq_norm(h,2));
    if(h_norm > EPSILON){
        h[0] /= h_norm;
        h[1] /= h_norm;
    }else{
        h[0] = 0;
        h[1] = 1;
    }
    h[0] = MIGRATION_BIAS*cos(0-my_position[2]) + (1-MIGRATION_BIAS)*h[0];
    h[1] = MIGRATION_BIAS*sin(0-my_position[2]) + (1-MIGRATION_BIAS)*h[1];
    h_norm = sqrtf(sq_norm(h,2));
    if(h_norm > EPSILON){
        h[0] /= h_norm;
        h[1] /= h_norm;
    }else{
        h[0] = 0;
        h[1] = 1;
    }

    // proximal vector
    int s;
    double value;
    double max_sens_value = 0;
    for(s = 0; s < NB_IR_SENSORS; s++){
        value = e_get_calibrated_prox(s);

        if(value > max_sens_value){
            max_sens_value = value;
        }
        float O_k = compute_O_k(value);
        float F_k = -O_k*O_k/C;

        p[0] += F_k*cos(sensorDir[s]);
        p[1] += F_k*sin(sensorDir[s]);
    }
    p[0] /= NB_IR_SENSORS;
    p[1] /= NB_IR_SENSORS;
    float norm_p = sqrtf(sq_norm(p, 2));
    if(norm_p > EPSILON) {
        p[0] /= norm_p;
        p[1] /= norm_p;
    }else{
        p[0] = 0;
        p[1] = 0;
    }


    // cohesion vector
    float denom = 1;
    float distance_to_cm = 0;
    for(id = 0; id < FLOCK_SIZE; id++){
        if(id==robot_id || (relative_pos[id][0] == 0 && relative_pos[id][1] == 0)) {
            continue;
        }
        c[0] += relative_pos[id][0];
        c[1] += relative_pos[id][1];
        denom++;
    }
    c[0] /= denom;
    c[1] /= denom;
    distance_to_cm = sqrtf(sq_norm(c, 2));
    if(distance_to_cm > EPSILON){
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

    float norm = sqrt(sq_norm(a, 2));
    if(norm > EPSILON){
        a[0] /= norm;
        a[1] /= norm;
    }else{
        a[0] = 0;
        a[1] = 1;
    }
    
    // speeds
    float phase_a = atan2(a[1],a[0]);

    float dot_c_h = 0;
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

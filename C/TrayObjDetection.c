#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "ToFSensor.h"

#include <pthread.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

#include <termios.h>
#include <math.h>

// trayMask: specifies ROI(ToF zones) where the signal is supposed to be reflected from the tray
int trayMask[TOF_NUM_SENSORS][TOF_NUM_ZONES] = {
    // sensor #1
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    // sensor #2
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    // sensor #3
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    // sensor #4
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    // sensor #5
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    // sensor #6
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 
     1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
}; // two dimensional array of zones to detect object existence.

// ----------------------------- sensor data simulation -----------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

#if 1 // code block mark: sensor data simulation
#define USE_TOF_SENSOR_SIMULATION 1

volatile sig_atomic_t stop_sensor_data_insert_flag = 1;
unsigned int rand_seed = 0;

int getRandomInt(int min, int max) {
    srand(rand_seed);
    int res = rand() % (max-min+1) + min;
    rand_seed = rand();
    return res;
}

void makeRandomToFSensorData(VL53L7CX_ResultsData *data) {
    //* Ambient noise in kcps/spads 
    #ifndef VL53L7CX_DISABLE_AMBIENT_PER_SPAD
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8; i++)
        data->ambient_per_spad[i] = getRandomInt(1, 100);
    #endif

    //* Number of valid target detected for 1 zone 
    #ifndef VL53L7CX_DISABLE_NB_TARGET_DETECTED
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8; i++)
        data->nb_target_detected[i] = getRandomInt(1, 100);
    #endif

    //* Number of spads enabled for this ranging 
    #ifndef VL53L7CX_DISABLE_NB_SPADS_ENABLED
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8; i++)
        data->nb_spads_enabled[i] = getRandomInt(1, 100);
    #endif

    //* Signal returned to the sensor in kcps/spads 
    #ifndef VL53L7CX_DISABLE_SIGNAL_PER_SPAD
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->signal_per_spad[i] = getRandomInt(1, 100);
    #endif

    //* Sigma of the current distance in mm 
    #ifndef VL53L7CX_DISABLE_RANGE_SIGMA_MM
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->range_sigma_mm[i] = getRandomInt(1, 100);
    #endif

    //* Measured distance in mm 
    #ifndef VL53L7CX_DISABLE_DISTANCE_MM
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->distance_mm[i] = getRandomInt(1, 100);
    #endif

    //* Estimated reflectance in percent 
    #ifndef VL53L7CX_DISABLE_REFLECTANCE_PERCENT
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->reflectance[i] = getRandomInt(1, 100);
    #endif

    //* Status indicating the measurement validity (5 & 9 means ranging OK)
    #ifndef VL53L7CX_DISABLE_TARGET_STATUS
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->target_status[i] = 5; // Range valid
    #endif
}

void insert_sensor_data() {
    if(!ToF_sensor_connected()) 
        printf("ToF sensor is not connected!\n");

    printf("inserting sensor data\n");
    for(int iSensor = 0; iSensor < TOF_NUM_SENSORS; iSensor++) {
        VL53L7CX_ResultsData data;
        makeRandomToFSensorData(&data);
        ToF_insert_sensor_data(iSensor, &data);
    }
}

void sensor_data_timer_callback(int signum) {
    insert_sensor_data();
}

void *sensor_data_thread(void *arg) {

    ToF_init_sensor_data();
    printf("init_sensor_data successful\n");

    int timer_interval_usec = (int) (1000*1000 / TOF_DATA_RATE);

    // start a timer
    struct itimerval timer;
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = timer_interval_usec;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = timer_interval_usec;

    setitimer(ITIMER_REAL, &timer, NULL);
    signal(SIGALRM, sensor_data_timer_callback);

    while(!stop_sensor_data_insert_flag) {
        ;
    }

    // stop timer
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = 0;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
    setitimer(ITIMER_REAL, &timer, NULL);

    ToF_free_sensor_data();
    printf("free_sensor_data successful\n");

    pthread_exit(NULL);
}

pthread_t start_sensor_simulation() {
    stop_sensor_data_insert_flag = 0;
    // start the tof sensor data simulation thread
    pthread_t sensor_simulation_thread_id;
    int rc;
    rc = pthread_create(&sensor_simulation_thread_id, NULL, sensor_data_thread, NULL);
    if (rc != 0) {
        fprintf(stderr, "Error creating thread: %d\n", rc);
        exit(EXIT_FAILURE);
    }

    return sensor_simulation_thread_id;
}

void stop_sensor_simulation() {
    stop_sensor_data_insert_flag = 1;
}

bool is_sensor_simulation_running() {
    return !stop_sensor_data_insert_flag;
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// ----------------------------- sensor data simulation -----------------------------
#endif


// --------------------------------- zero adjustment --------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


#if 1 // code block mark: zero adjustment

ToFSensorData_Common zeroData_common[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA];
ToFSensorData_Common currentData_common[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA];
ToFSensorData_Target zeroData_target[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_TPZ][TOF_NUM_DATA];
ToFSensorData_Target currentData_target[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_TPZ][TOF_NUM_DATA];

void printZeroData(int field_to_print) {
    printf("printZeroData: This is for debug\n");
    for(int i = 0; i < 1; i++) { // loop for sensors
        for(int j = 0; j < TOF_NUM_ZONES; j++) { // loop for data
            switch(field_to_print) {
  //* Ambient noise in kcps/spads 
#ifndef VL53L7CX_DISABLE_AMBIENT_PER_SPAD
                case 0:
                    for(int k = 0; k < TOF_NUM_DATA; k++) {
                        printf("0x%02x,", zeroData_common[i][j][k].ambient_per_spad);
                    }
                    printf("\n");
                    break;
#endif

  //* Number of valid target detected for 1 zone 
#ifndef VL53L7CX_DISABLE_NB_TARGET_DETECTED
                case 1:
                    for(int k = 0; k < TOF_NUM_DATA; k++) {
                        printf("0x%02x,", zeroData_common[i][j][k].nb_target_detected);
                    }
                    printf("\n");
                    break;
#endif

  //* Number of spads enabled for this ranging 
#ifndef VL53L7CX_DISABLE_NB_SPADS_ENABLED
                case 2:
                    for(int k = 0; k < TOF_NUM_DATA; k++) {
                        printf("0x%02x,", zeroData_common[i][j][k].nb_spads_enabled);
                    }
                    printf("\n");
                    break;
#endif

  //* Signal returned to the sensor in kcps/spads 
#ifndef VL53L7CX_DISABLE_SIGNAL_PER_SPAD
                case 3:
                    for(int k = 0; k < TOF_NUM_TPZ; k++) {
                        for(int l = 0; l < TOF_NUM_DATA; l++)
                            printf("0x%02x,", zeroData_target[i][j][k][l].signal_per_spad);
                    }
                    printf("\n");
                    break;
#endif

  //* Sigma of the current distance in mm 
#ifndef VL53L7CX_DISABLE_RANGE_SIGMA_MM
                case 4:
                    for(int k = 0; k < TOF_NUM_TPZ; k++) {
                        for(int l = 0; l < TOF_NUM_DATA; l++)
                            printf("0x%02x,", zeroData_target[i][j][k][l].range_sigma_mm);
                    }
                    printf("\n");
                    break;
#endif

  //* Measured distance in mm 
#ifndef VL53L7CX_DISABLE_DISTANCE_MM
                case 5:
                    for(int k = 0; k < TOF_NUM_TPZ; k++) {
                        for(int l = 0; l < TOF_NUM_DATA; l++)
                            printf("0x%02x,", zeroData_target[i][j][k][l].distance_mm);
                    }
                    printf("\n");
                    break;
#endif

  //* Estimated reflectance in percent 
#ifndef VL53L7CX_DISABLE_REFLECTANCE_PERCENT
                case 6:
                    for(int k = 0; k < TOF_NUM_TPZ; k++) {
                        for(int l = 0; l < TOF_NUM_DATA; l++)
                            printf("0x%02x,", zeroData_target[i][j][k][l].reflectance);
                    }
                    printf("\n");
                    break;
#endif

  //* Status indicating the measurement validity (5 & 9 means ranging OK)
#ifndef VL53L7CX_DISABLE_TARGET_STATUS
                case 7:
                    for(int k = 0; k < TOF_NUM_TPZ; k++) {
                        for(int l = 0; l < TOF_NUM_DATA; l++)
                            printf("0x%02x,", zeroData_target[i][j][k][l].target_status);
                    }
                    printf("\n");
                    break;
#endif

                default:
                    break;
            }
            printf("\n");
        }
    }
}

void doZeroAdjustment() {
    bool sensor_simulation_running = is_sensor_simulation_running();
    if(!sensor_simulation_running) {
        printf("run the simulation first\n");
        return;
    } else {
        ToF_clear_sensor_data();
    }

    while(!ToF_is_sensor_data_full()) {
        ;
    }

    printf("enough zero data was cumulated\n");
}

void acquireZeroData() {
    if(is_sensor_simulation_running()) 
        ToF_copy_sensor_data(zeroData_common, zeroData_target);
}

void *zero_adjustment_thread_function() {
    printf("started doing zero adjustment\n");
    printZeroData(0);
    doZeroAdjustment();
    acquireZeroData();
    printZeroData(0);

    printf("finished doing zero adjustment\n");
    pthread_exit(NULL);
}

void startZeroAdjustment() {
    pthread_t zero_adjustment_thread_id;
    int rc;
    rc = pthread_create(&zero_adjustment_thread_id, NULL, zero_adjustment_thread_function, NULL);
    if (rc != 0) {
        fprintf(stderr, "Error creating thread: %d\n", rc);
        exit(EXIT_FAILURE);
    }
}



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// --------------------------------- zero adjustment --------------------------------
#endif


// -------------------------------- object detection --------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


#if 1 // code block mark: object detection

double calculateMean(const double data[], int size) {
    double sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return sum / size;
}

double calculateVariance(const double data[], int size, double mean) {
    double sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += pow(data[i] - mean, 2);
    }
    return sum / (size - 1);
}

double calculateTStatistic(const double data1[], int size1, const double data2[], int size2) {
    double mean1 = calculateMean(data1, size1);
    double mean2 = calculateMean(data2, size2);

    double variance1 = calculateVariance(data1, size1, mean1);
    double variance2 = calculateVariance(data2, size2, mean2);

    double pooledVariance = ((size1 - 1) * variance1 + (size2 - 1) * variance2) / (size1 + size2 - 2);

    double tStatistic = (mean1 - mean2) / sqrt(pooledVariance * (1.0 / size1 + 1.0 / size2));

    return tStatistic;
}

// t-test용 critical value는 자유도 및 유의수준에 따른 값 미리 테이블로 만들어 놓고 읽어 오기
// criticalValues[one-two-tail][dof][sig]

typedef enum {
    TTEST_ONETAILED,
    TTEST_TWOTAILED,
    TTEST_TYPE_COUNT
} TTEST_ONETWOTAIL;

// dof: (16, 64) x (1~4) x 2 -2: 30, 62, 94, 126, 254, 382, 510
typedef enum {
    DOF_30,
    DOF_62,
    DOF_94,
    DOF_126,
    DOF_254,
    DOF_382,
    DOF_510,
    DOF_COUNT
} DOF;

typedef enum {
    SIG_LEVEL_02000,
    SIG_LEVEL_01500,
    SIG_LEVEL_01000,
    SIG_LEVEL_00500,
    SIG_LEVEL_00250,
    SIG_LEVEL_00100,
    SIG_LEVEL_00050,
    SIG_LEVEL_00010,
    SIG_LEVEL_00005,
    SIG_LEVEL_COUNT
} SIG_LEVEL;

const double criticalValues[TTEST_TYPE_COUNT][DOF_COUNT][SIG_LEVEL_COUNT] = {
	{
        {0.853767, 1.05466, 1.31042, 1.69726, 2.04227, 2.45726, 2.75, 3.38518, 3.64596},
        {0.847457, 1.04518, 1.29536, 1.6698, 1.99897, 2.38801, 2.65748, 3.22696, 3.45448},
        {0.845462, 1.04218, 1.29062, 1.66123, 1.98552, 2.36667, 2.62915, 3.17921, 3.39706},
        {0.844483, 1.04072, 1.28831, 1.65704, 1.97897, 2.35631, 2.61541, 3.15617, 3.36942},
        {0.843039, 1.03855, 1.28489, 1.65087, 1.96935, 2.34112, 2.59532, 3.12263, 3.32924},
        {0.842563, 1.03784, 1.28377, 1.64885, 1.96619, 2.33615, 2.58876, 3.1117, 3.31618},
        {0.842327, 1.03749, 1.28321, 1.64785, 1.96463, 2.33368, 2.5855, 3.10629, 3.30971}
	},
	{
        {1.31042, 1.47736, 1.69726, 2.04227, 2.35956, 2.75, 3.0298, 3.64596, 3.90164},
        {1.29536, 1.45759, 1.6698, 1.99897, 2.29714, 2.65748, 2.91097, 3.45448, 3.67394},
        {1.29062, 1.45139, 1.66123, 1.98552, 2.27787, 2.62915, 2.8748, 3.39706, 3.60608},
        {1.28831, 1.44836, 1.65704, 1.97897, 2.2685, 2.61541, 2.85731, 3.36942, 3.5735},
        {1.28489, 1.4439, 1.65087, 1.96935, 2.25477, 2.59532, 2.83177, 3.32924, 3.52622},
        {1.28377, 1.44243, 1.64885, 1.96619, 2.25027, 2.58876, 2.82343, 3.31618, 3.51087},
        {1.28321, 1.4417, 1.64785, 1.96463, 2.24804, 2.5855, 2.8193, 3.30971, 3.50327}
	}
};

// Function to calculate the critical value for a t-test
double calculateCriticalValue(double significanceLevel, int degreesOfFreedom, bool twoTailed) {
    // Create a students_t distribution with the given degrees of freedom
    // boost::math::students_t dist(degreesOfFreedom);

    // // Calculate the critical value using the inverse cumulative distribution function (quantile function)
    // double t_critical;
    // if (twoTailed) {
    //     significanceLevel = significanceLevel / 2.0; // Adjust the significance level for a two-tailed test
    // }

    // t_critical = boost::math::quantile(dist, 1 - significanceLevel);

    TTEST_ONETWOTAIL onetwo_tail = (twoTailed ? TTEST_TWOTAILED : TTEST_ONETAILED);
    DOF dof;
    if (degreesOfFreedom < 50) {
        dof = DOF_30;
    } else if (degreesOfFreedom < 70) {
        dof = DOF_62;
    } else if (degreesOfFreedom < 100) {
        dof = DOF_94;
    } else if (degreesOfFreedom < 200) {
        dof = DOF_126;
    } else if (degreesOfFreedom < 320) {
        dof = DOF_254;
    } else if (degreesOfFreedom < 450) {
        dof = DOF_382;
    } else {
        dof = DOF_510;
    }

    SIG_LEVEL sig_level;
    if (significanceLevel <= 0.0005) {
        sig_level = SIG_LEVEL_00005;
    } else if (significanceLevel <= 0.001) {
        sig_level = SIG_LEVEL_00010;
    } else if (significanceLevel <= 0.01) {
        sig_level = SIG_LEVEL_00100;
    } else if (significanceLevel <= 0.025) {
        sig_level = SIG_LEVEL_00250;
    } else if (significanceLevel < 320) {
        sig_level = SIG_LEVEL_00500;
    } else if (significanceLevel < 450) {
        sig_level = SIG_LEVEL_01000;
    } else if (significanceLevel < 450) {
        sig_level = SIG_LEVEL_01500;
    } else {
        sig_level = SIG_LEVEL_02000;
    }

    return criticalValues[onetwo_tail][dof][sig_level];
}

int detectObjectOnTray(ToFSensorData_Target *current, ToFSensorData_Target *zero) {

    int size1 = TOF_NUM_DATA;
    int size2 = TOF_NUM_DATA;

    double dist1[TOF_NUM_DATA];
    double dist2[TOF_NUM_DATA];
    for(int i = 0; i < TOF_NUM_DATA; i++) {
        dist1[i] = (double)current[i].distance_mm;
        dist2[i] = (double)zero[i].distance_mm;
    }
    double tStatistic = calculateTStatistic(dist1, size1, dist2, size2);

    double significanceLevel = 0.05;  // Significance level (e.g., 0.05 for 5%)
    int degreesOfFreedom = size1 + size2 - 2;        // Degrees of freedom
    bool twoTailed = false;            // Whether the test is two-tailed (true) or one-tailed (false)
    double criticalValue = calculateCriticalValue(significanceLevel, degreesOfFreedom, twoTailed);

    printf("Significance Level: %f\n", significanceLevel);
    printf("Degrees of Freedom: %d\n", degreesOfFreedom);
    printf("Critical value for %s tset: %f\n", (twoTailed?"Two Tailed":"One Tailed"), criticalValue);

    // Check if the tStatistic is less than (-1 * criticalValue) for H0 rejection
    bool significantDifference = (tStatistic < -criticalValue);

    printf("T-Statistic: %f\n", tStatistic);
    printf("Significant Difference: %s\n", (significantDifference ? "Yes" : "No"));

    return significantDifference;
}

void startObjectDetection() {
    if(!is_sensor_simulation_running()) {
        printf("ToF sensor must be running to detect object.\n");
        return;
    }

    printf("startObjectDetection\n");
    if(is_sensor_simulation_running()) 
        ToF_copy_sensor_data(currentData_common, currentData_target);

    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        for(int j = 0; j < TOF_NUM_ZONES; j++) {
            for(int k = 0; k < TOF_NUM_TPZ; k++) {
                bool objFound = detectObjectOnTray(currentData_target[i][j][k], zeroData_target[i][j][k]);
                if(objFound)
                    printf("Object found at sensor %d, zone %d, target %d\n", i, j, k);
            }
        }
    }
}

#endif


// -------------------------------- main application --------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


// Function to read a key without waiting for Enter
char getch()
{
    struct termios oldAttr, newAttr;
    char key;

    tcgetattr(0, &oldAttr);
    newAttr = oldAttr;
    newAttr.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &newAttr);

    key = getchar();

    tcsetattr(0, TCSANOW, &oldAttr);
    return key;
}

void sigint_handler(int signum) {
    printf("sigint_handler\n");
#if USE_TOF_SENSOR_SIMULATION
    stop_sensor_data_insert_flag = 1;
#endif
    // exit(EXIT_FAILURE);
}

#if USE_TOF_SENSOR_SIMULATION
const char *help_message = "\n\
press one of the following keys to do what you want:\n\
    q: quit this program\n\
    0: zero adjustment\n\
    d: run object detectection on the tray\n\
    s: start or stop generating and inserting sensor data for simulation\n\
    h: display this help message\n\
";
#else
const char *help_message = "\n\
press one of the following keys to do what you want:\n\
    q: quit this program\n\
    0: zero adjustment\n\
    d: run object detectection on the tray\n\
    h: display this help message\n\
";
#endif

int main(int argc, char** argv) {
    signal(SIGINT, sigint_handler);

    printf("%s\n", help_message);
    while(1) {
        char key = getch();
        switch (key) {
            case '0':
                //do zero adjustment 
                printf("start zero adjustment\n");
                startZeroAdjustment();
                break;
            case 'd':
                // start detection
                printf("start detecting objects on the tray\n");
                startObjectDetection();
                break;
            case 'h':
                printf("%s\n", help_message);
                break;
#if USE_TOF_SENSOR_SIMULATION
            case 's':
                if(!stop_sensor_data_insert_flag) {
                    printf("generating and inserting sensor data stopped\n");
                    stop_sensor_simulation();
                } else {
                    printf("generating and inserting sensor data started\n");
                    start_sensor_simulation();
                }
                break;
#endif
            default:
                break;
        }

        if (key == 'q') {
#if USE_TOF_SENSOR_SIMULATION
            stop_sensor_data_insert_flag = 1;
#endif
            break;
        }
    }

    printf("terminating the program ......\n");

    return 0;
}

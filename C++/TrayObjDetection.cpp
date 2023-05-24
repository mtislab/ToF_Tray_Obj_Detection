#include <iostream>
#include <iomanip>
#include <thread>
#include "ToFSensor.hpp"

// #include <unistd.h>
// #include <signal.h>
#include <csignal>
#include <sys/time.h>

#include <termios.h>

#include <cmath>
#include <algorithm>
#include <boost/math/distributions/students_t.hpp>

using namespace std;


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

ToFSensor *the_sensor = NULL;

volatile sig_atomic_t stop_sensor_data_insert_flag = 1;
unsigned int rand_seed = 0;
int getRandomInt(int min=1, int max=100) {
    srand(rand_seed);
    int res = rand() % (max-min+1) + min;
    rand_seed = rand();
    return res;
}

void makeRandomToFSensorData(VL53L7CX_ResultsData *data) {
    //* Ambient noise in kcps/spads 
    #ifndef VL53L7CX_DISABLE_AMBIENT_PER_SPAD
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8; i++)
        data->ambient_per_spad[i] = getRandomInt();
    #endif

    //* Number of valid target detected for 1 zone 
    #ifndef VL53L7CX_DISABLE_NB_TARGET_DETECTED
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8; i++)
        data->nb_target_detected[i] = getRandomInt();
    #endif

    //* Number of spads enabled for this ranging 
    #ifndef VL53L7CX_DISABLE_NB_SPADS_ENABLED
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8; i++)
        data->nb_spads_enabled[i] = getRandomInt();
    #endif

    //* Signal returned to the sensor in kcps/spads 
    #ifndef VL53L7CX_DISABLE_SIGNAL_PER_SPAD
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->signal_per_spad[i] = getRandomInt();
    #endif

    //* Sigma of the current distance in mm 
    #ifndef VL53L7CX_DISABLE_RANGE_SIGMA_MM
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->range_sigma_mm[i] = getRandomInt();
    #endif

    //* Measured distance in mm 
    #ifndef VL53L7CX_DISABLE_DISTANCE_MM
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->distance_mm[i] = getRandomInt();
    #endif

    //* Estimated reflectance in percent 
    #ifndef VL53L7CX_DISABLE_REFLECTANCE_PERCENT
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->reflectance[i] = getRandomInt();
    #endif

    //* Status indicating the measurement validity (5 & 9 means ranging OK)
    #ifndef VL53L7CX_DISABLE_TARGET_STATUS
    for(int i = 0; i < VL53L7CX_RESOLUTION_8X8*VL53L7CX_NB_TARGET_PER_ZONE; i++)
        data->target_status[i] = getRandomInt();
    #endif
}

void insert_sensor_data() {
    if(!the_sensor) cout << "ToF sensor is not connected!" << endl;
    printf("inserting sensor data\n");
    for(int iSensor = 0; iSensor < TOF_NUM_SENSORS; iSensor++) {
        VL53L7CX_ResultsData data;
        makeRandomToFSensorData(&data);
        the_sensor->insert_sensor_data(iSensor, &data);
    }
}

void sensor_data_timer_callback(int signum) {
    insert_sensor_data();
}

void sensorSimulationThreadFunction(int arg1 = 0, int arg2 = 0) {

    int timer_interval_usec = (int) (1000*1000 / TOF_DATA_RATE);

    // Set up the signal handler
    struct sigaction sa{};
    sa.sa_handler = sensor_data_timer_callback;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGALRM, &sa, nullptr);

    // Set the timer interval
    struct itimerval timer{};
    timer.it_value.tv_sec = 0;  // Initial delay
    timer.it_value.tv_usec = timer_interval_usec;
    timer.it_interval.tv_sec = 0;  // Interval
    timer.it_interval.tv_usec = timer_interval_usec;  // Interval

    // Start the timer
    setitimer(ITIMER_REAL, &timer, nullptr);

    while(!stop_sensor_data_insert_flag) {
        ;
    }

    // stop timer
    timer.it_value.tv_sec = 0;
    timer.it_value.tv_usec = 0;
    timer.it_interval.tv_sec = 0;
    timer.it_interval.tv_usec = 0;
    setitimer(ITIMER_REAL, &timer, NULL);
}

void start_sensor_simulation() {
    stop_sensor_data_insert_flag = 0;
    if(the_sensor == NULL) {
        the_sensor = new ToFSensor();
    }
    // start the tof sensor data simulation thread
    thread sensorSimulator([&]() {
        sensorSimulationThreadFunction();
    });
    sensorSimulator.detach();
}

void stop_sensor_simulation() {
    stop_sensor_data_insert_flag = 1;
    if(the_sensor != NULL) {
        delete the_sensor;
        the_sensor = NULL;
    }
}

bool is_sensor_simulation_running() {
    return !stop_sensor_data_insert_flag;
}

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// ----------------------------- sensor data simulation -----------------------------
#endif

// --------------------------------- zero adjustment --------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

ToFSensorData zeroData_common[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA];
ToFSensorData currentData[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA];

void printZeroData(int field_to_print) {
    cout << "printZeroData: This is for debug" << endl;
    for(int i = 0; i < 1; i++) { // loop for sensors
        for(int j = 0; j < TOF_NUM_DATA; j++) { // loop for data
            switch(field_to_print) {
  //* Ambient noise in kcps/spads 
#ifndef VL53L7CX_DISABLE_AMBIENT_PER_SPAD
                case 0:
                    for(int k = 0; k < VL53L7CX_RESOLUTION_8X8; k++) {
                        cout << "0x" << hex << setfill('0') << setw(2) << zeroData_common[i][j][k].ambient_per_spad << ",";
                    }
                    cout << endl;
                    break;
#endif

  //* Number of valid target detected for 1 zone 
#ifndef VL53L7CX_DISABLE_NB_TARGET_DETECTED
                case 1:
                    for(int k = 0; k < VL53L7CX_RESOLUTION_8X8; k++) {
                        cout << "0x" << hex << setfill('0') << setw(2) << zeroData_common[i][j][k].nb_target_detected << ",";
                    }
                    cout << endl;
                    break;
#endif

  //* Number of spads enabled for this ranging 
#ifndef VL53L7CX_DISABLE_NB_SPADS_ENABLED
                case 2:
                    for(int k = 0; k < VL53L7CX_RESOLUTION_8X8; k++) {
                        cout << "0x" << hex << setfill('0') << setw(2) << zeroData_common[i][j][k].nb_spads_enabled << ",";
                    }
                    cout << endl;
                    break;
#endif

  //* Signal returned to the sensor in kcps/spads 
#ifndef VL53L7CX_DISABLE_SIGNAL_PER_SPAD
                case 3:
                    for(int k = 0; k < VL53L7CX_RESOLUTION_8X8 * VL53L7CX_NB_TARGET_PER_ZONE; k++) {
                        cout << "0x" << hex << setfill('0') << setw(2) << zeroData_common[i][j][k].signal_per_spad << ",";
                    }
                    cout << endl;
                    break;
#endif

  //* Sigma of the current distance in mm 
#ifndef VL53L7CX_DISABLE_RANGE_SIGMA_MM
                case 4:
                    for(int k = 0; k < VL53L7CX_RESOLUTION_8X8 * VL53L7CX_NB_TARGET_PER_ZONE; k++) {
                        cout << "0x" << hex << setfill('0') << setw(2) << zeroData_common[i][j][k].range_sigma_mm << ",";
                    }
                    cout << endl;
                    break;
#endif

  //* Measured distance in mm 
#ifndef VL53L7CX_DISABLE_DISTANCE_MM
                case 5:
                    for(int k = 0; k < VL53L7CX_RESOLUTION_8X8 * VL53L7CX_NB_TARGET_PER_ZONE; k++) {
                        cout << "0x" << hex << setfill('0') << setw(2) << zeroData_common[i][j][k].distance_mm << ",";
                    }
                    cout << endl;
                    break;
#endif

  //* Estimated reflectance in percent 
#ifndef VL53L7CX_DISABLE_REFLECTANCE_PERCENT
                case 6:
                    for(int k = 0; k < VL53L7CX_RESOLUTION_8X8 * VL53L7CX_NB_TARGET_PER_ZONE; k++) {
                        cout << "0x" << hex << setfill('0') << setw(2) << zeroData_common[i][j][k].reflectance << ",";
                    }
                    cout << endl;
                    break;
#endif

  //* Status indicating the measurement validity (5 & 9 means ranging OK)
#ifndef VL53L7CX_DISABLE_TARGET_STATUS
                case 7:
                    for(int k = 0; k < VL53L7CX_RESOLUTION_8X8 * VL53L7CX_NB_TARGET_PER_ZONE; k++) {
                        cout << "0x" << hex << setfill('0') << setw(2) << zeroData_common[i][j][k].target_status << ",";
                    }
                    cout << endl;
                    break;
#endif

                default:
                    break;
            }
            cout << endl;
        }
    }
}

void doZeroAdjustment() {
    // maybe it is better to add some flag to ToFSensor to specify that the zero adjustment status is in progress
    if(the_sensor)
        the_sensor->clear_sensor_data();

    bool sensor_simulation_running = is_sensor_simulation_running();
    if(!sensor_simulation_running) {
        cout << "run the simulation first" << endl;
        return;
    }

    while(!the_sensor->is_sensor_data_full()) {
        ;
    }

    cout << "enough zero data was cumulated" << endl;
}

void acquireZeroData() {
    if(the_sensor) 
        the_sensor->copy_sensor_data(zeroData_common);
}

void zero_adjustment_thread_function(int someArg=0) {
    cout << "started doing zero adjustment" << endl;
    printZeroData(0);
    doZeroAdjustment();
    acquireZeroData();
    printZeroData(0);

    cout << "finished doing zero adjustment" << endl;
}

void startZeroAdjustment() {
    thread zeroAdjustmentThread([&]() {
        zero_adjustment_thread_function();
    });
    zeroAdjustmentThread.detach();
}



// Object Detection

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
// double criticalValues[][] = {};

// Function to calculate the critical value for a t-test
double calculateCriticalValue(double significanceLevel, int degreesOfFreedom, bool twoTailed) {
    // Create a students_t distribution with the given degrees of freedom
    boost::math::students_t dist(degreesOfFreedom);

    // Calculate the critical value using the inverse cumulative distribution function (quantile function)
    double t_critical;
    if (twoTailed) {
        significanceLevel = significanceLevel / 2.0; // Adjust the significance level for a two-tailed test
    }

    t_critical = boost::math::quantile(dist, 1 - significanceLevel);

    return t_critical;
}

int detectObjectOnTray(int iSeonsor, ToFSensorData current[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA], ToFSensorData zero[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA]) {

    int size1 = TOF_NUM_DATA;
    int size2 = TOF_NUM_DATA;

    for(int i = 0; i < TOF_NUM_ZONES; i++) {

        double dist1[TOF_NUM_DATA];
        double dist2[TOF_NUM_DATA];
        for(int j = 0; j < TOF_NUM_DATA; j++) {
            dist1[j] = (double)current[iSeonsor][i][j].distance_mm[0];
            dist2[j] = (double)zero[iSeonsor][i][j].distance_mm[0];
        }
        double tStatistic = calculateTStatistic(dist1, size1, dist2, size2);

        double significanceLevel = 0.05;  // Significance level (e.g., 0.05 for 5%)
        int degreesOfFreedom = size1 + size2 - 2;        // Degrees of freedom
        bool twoTailed = false;            // Whether the test is two-tailed (true) or one-tailed (false)
        double criticalValue = calculateCriticalValue(significanceLevel, degreesOfFreedom, twoTailed);

        std::cout << "Significance Level: " << significanceLevel << std::endl;
        std::cout << "Degrees of Freedom: " << degreesOfFreedom << std::endl;
        std::cout << "Critical value for " << (twoTailed?"Two Tailed":"One Tailed") << " test: " << criticalValue << std::endl;

        // Check if the tStatistic is less than (-1 * criticalValue) for H0 rejection
        bool significantDifference = tStatistic < -criticalValue;

        std::cout << "T-Statistic: " << tStatistic << std::endl;
        std::cout << "Significant Difference: " << (significantDifference ? "Yes" : "No") << std::endl;
        if (significantDifference)
            std::cout << "Object found in zone " << i << std::endl;
    }

    return 0;
}

void startObjectDetection() {
    if(the_sensor == NULL) {
        cout << "ToF sensor must be running to detect object." << endl;
        return;
    }

    cout << "startObjectDetection" << endl;
    if(the_sensor) 
        the_sensor->copy_sensor_data(currentData);

    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        detectObjectOnTray(i, currentData, zeroData_common);
    }
}

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
                cout << "start zero adjustment" << endl;
                startZeroAdjustment();
                break;
            case 'd':
                // start detection
                cout << "start detecting objects on the tray" << endl;
                startObjectDetection();
                break;
            case 'h':
                cout << help_message << endl;
                break;
#if USE_TOF_SENSOR_SIMULATION
            case 's':
                if(!stop_sensor_data_insert_flag) {
                    cout << "generating and inserting sensor data stopped" << endl;
                    stop_sensor_simulation();
                } else {
                    cout << "generating and inserting sensor data started" << endl;
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

    cout << "terminating the program ......\n";

    return 0;
}

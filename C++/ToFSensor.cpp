#include <iostream>
#include <cstring>
#include "ToFSensor.hpp"

using namespace std;

ToFSensorData ToFSensor::default_data = 
{
  //* Ambient noise in kcps/spads 
#ifndef VL53L7CX_DISABLE_AMBIENT_PER_SPAD
  //uint32_t ambient_per_spad;
  0,
#endif

  //* Number of valid target detected for 1 zone 
#ifndef VL53L7CX_DISABLE_NB_TARGET_DETECTED
  //uint8_t nb_target_detected;
  0,
#endif

  //* Number of spads enabled for this ranging 
#ifndef VL53L7CX_DISABLE_NB_SPADS_ENABLED
  //uint32_t nb_spads_enabled;
  0,
#endif

  //* Signal returned to the sensor in kcps/spads 
#ifndef VL53L7CX_DISABLE_SIGNAL_PER_SPAD
  //uint32_t signal_per_spad[VL53L7CX_NB_TARGET_PER_ZONE];
  {0},
#endif

  //* Sigma of the current distance in mm 
#ifndef VL53L7CX_DISABLE_RANGE_SIGMA_MM
//   uint16_t range_sigma_mm[VL53L7CX_NB_TARGET_PER_ZONE];
  {0},
#endif

  //* Measured distance in mm 
#ifndef VL53L7CX_DISABLE_DISTANCE_MM
//   int16_t distance_mm[VL53L7CX_NB_TARGET_PER_ZONE];
  {0},
#endif

  //* Estimated reflectance in percent 
#ifndef VL53L7CX_DISABLE_REFLECTANCE_PERCENT
//   uint8_t reflectance[VL53L7CX_NB_TARGET_PER_ZONE];
  {0},
#endif

  //* Status indicating the measurement validity (5 & 9 means ranging OK)
#ifndef VL53L7CX_DISABLE_TARGET_STATUS
//   uint8_t target_status[VL53L7CX_NB_TARGET_PER_ZONE];
  {0},
#endif
};

// constructor
ToFSensor::ToFSensor() {
    ToFSensorName = "VL53L7CX";
    init_sensor_data();
}

ToFSensor::~ToFSensor() {
    free_sensor_data();
}

#if USE_DYNAMIC_ALLOC_FOR_SENSOR_DATA

void ToFSensor::init_sensor_data() {
    the_sensor_data_common = (ToFSensorData***) calloc(TOF_NUM_SENSORS, sizeof(ToFSensorData*));
    if(the_sensor_data_common == NULL) {
        cout << "Memory allocation failed.\n";
        exit(1);
    }

    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        the_sensor_data_common[i] = (ToFSensorData**) calloc(TOF_NUM_ZONES, sizeof(ToFSensorData*));
        if(the_sensor_data_common[i] == NULL) {
            cout << "Memory allocation failed.\n";
            exit(1);
        }

        for(int j = 0; j < TOF_NUM_ZONES; j++) {
            the_sensor_data_common[i][j] = (ToFSensorData*) calloc(TOF_NUM_DATA, sizeof(ToFSensorData));
            if(the_sensor_data_common[i][j] == NULL) {
                cout << "Memory allocation failed.\n";
                exit(1);
            }

            // // already zero initialized by calloc: no need for the following default_data assignment
            // for(int k = 0; k < TOF_NUM_DATA; k++) {
            //     the_sensor_data_common[i][j][k] = default_data;
            // }
        }
    }

    memset(sensor_data_head, 0, TOF_NUM_SENSORS);
    memset(sensor_data_tail, -1, TOF_NUM_SENSORS);

    cout << "init_sensor_data successful\n";
}

void ToFSensor::free_sensor_data() {
    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        for(int j = 0; j < TOF_NUM_ZONES; j++) {
            if(the_sensor_data_common[i][j] != NULL)
                free(the_sensor_data_common[i][j]);
        }

        if(the_sensor_data_common[i] != NULL)
            free(the_sensor_data_common[i]);
    }

    if(the_sensor_data_common != NULL)
        free(the_sensor_data_common);

    cout << "free_sensor_data successful\n";
}

#else

void ToFSensor::init_sensor_data() {
    // for(int i = 0; i < TOF_NUM_SENSORS; i++) {
    //     for(int j = 0; j < TOF_NUM_DATA; j++) {
    //         for(int k = 0; k < TOF_NUM_ZONES; k++) {
    //             the_sensor_data_common[i][j][k] = default_data;
    //         }
    //     }
    // }
    memset(the_sensor_data_common, 0, sizeof(ToFSensorData) * TOF_NUM_SENSORS * TOF_NUM_ZONES * TOF_NUM_DATA);

    memset(sensor_data_head, 0, TOF_NUM_SENSORS);
    memset(sensor_data_tail, -1, TOF_NUM_SENSORS);

    cout << "init_sensor_data successful\n";
}

void ToFSensor::free_sensor_data() {
    // no need to do anything here.

    cout << "free_sensor_data successful\n";
}

#endif

void ToFSensor::clear_sensor_data() {
    if(the_sensor_data_common == NULL) {
        init_sensor_data();
    } else {
        for(int i = 0; i < TOF_NUM_SENSORS; i++) {
            for(int j = 0; j < TOF_NUM_ZONES; j++) {
                for(int k = 0; k < TOF_NUM_DATA; k++) {
                    the_sensor_data_common[i][j][k] = default_data;
                }
            }
        }

        memset(sensor_data_head, 0, TOF_NUM_SENSORS);
        memset(sensor_data_tail, -1, TOF_NUM_SENSORS);
    }

    cout << "sensor data clearing successful" << endl;
}

// always insert a sensor data at the end
void ToFSensor::insert_sensor_data(int iSensor, VL53L7CX_ResultsData* sensor_data) {
    // get current head and tail
    int current_head = sensor_data_head[iSensor]; // first data position
    int current_tail = sensor_data_tail[iSensor]; // last data position

    // calc next tail(data input position) and insert data
    int next_tail = (current_tail + 1) % TOF_NUM_DATA;

    for(int i = 0; i < TOF_NUM_ZONES; i++) {
        //* Ambient noise in kcps/spads 
        #ifndef VL53L7CX_DISABLE_AMBIENT_PER_SPAD
        the_sensor_data_common[iSensor][i][next_tail].ambient_per_spad = sensor_data->ambient_per_spad[i];
        #endif

        //* Number of valid target detected for 1 zone 
        #ifndef VL53L7CX_DISABLE_NB_TARGET_DETECTED
        the_sensor_data_common[iSensor][i][next_tail].nb_target_detected = sensor_data->nb_target_detected[i];
        #endif

        //* Number of spads enabled for this ranging 
        #ifndef VL53L7CX_DISABLE_NB_SPADS_ENABLED
        the_sensor_data_common[iSensor][i][next_tail].nb_spads_enabled = sensor_data->nb_spads_enabled[i];
        #endif

        //* Signal returned to the sensor in kcps/spads 
        #ifndef VL53L7CX_DISABLE_SIGNAL_PER_SPAD
        memcpy(the_sensor_data_common[iSensor][i][next_tail].signal_per_spad, sensor_data->signal_per_spad + i * VL53L7CX_NB_TARGET_PER_ZONE, sizeof(uint32_t) * VL53L7CX_NB_TARGET_PER_ZONE);
        #endif

        //* Sigma of the current distance in mm 
        #ifndef VL53L7CX_DISABLE_RANGE_SIGMA_MM
        memcpy(the_sensor_data_common[iSensor][i][next_tail].range_sigma_mm, sensor_data->range_sigma_mm + i * VL53L7CX_NB_TARGET_PER_ZONE, sizeof(uint16_t) * VL53L7CX_NB_TARGET_PER_ZONE);
        #endif

        //* Measured distance in mm 
        #ifndef VL53L7CX_DISABLE_DISTANCE_MM
        memcpy(the_sensor_data_common[iSensor][i][next_tail].distance_mm, sensor_data->distance_mm + i * VL53L7CX_NB_TARGET_PER_ZONE, sizeof(uint16_t) * VL53L7CX_NB_TARGET_PER_ZONE);
        #endif

        //* Estimated reflectance in percent 
        #ifndef VL53L7CX_DISABLE_REFLECTANCE_PERCENT
        memcpy(the_sensor_data_common[iSensor][i][next_tail].reflectance, sensor_data->reflectance + i * VL53L7CX_NB_TARGET_PER_ZONE, sizeof(uint8_t) * VL53L7CX_NB_TARGET_PER_ZONE);
        uint8_t reflectance[VL53L7CX_NB_TARGET_PER_ZONE];
        #endif

        //* Status indicating the measurement validity (5 & 9 means ranging OK)
        #ifndef VL53L7CX_DISABLE_TARGET_STATUS
        memcpy(the_sensor_data_common[iSensor][i][next_tail].target_status, sensor_data->target_status + i * VL53L7CX_NB_TARGET_PER_ZONE, sizeof(uint8_t) * VL53L7CX_NB_TARGET_PER_ZONE);
        #endif
    }

    // set next head and tail position
    bool sensor_data_full = (next_tail == current_head && current_tail != -1);
    if (sensor_data_full) {
        current_head = (current_head + 1) % TOF_NUM_DATA;
        sensor_data_head[iSensor] = current_head;
    }
    sensor_data_tail[iSensor] = next_tail;
}

void ToFSensor::copy_sensor_data(ToFSensorData dest[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA]) {
#if USE_DYNAMIC_ALLOC_FOR_SENSOR_DATA
    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        for(int j = 0; j < TOF_NUM_ZONES; j++) {
            for(int k = 0; k < TOF_NUM_DATA; k++) {
                memcpy(dest[i][j]+k, the_sensor_data_common[i][j]+k, sizeof(ToFSensorData));
            }
        }
    }
#else
    memcpy(dest, the_sensor_data_common, sizeof(ToFSensorData) * TOF_NUM_SENSORS * TOF_NUM_ZONES * TOF_NUM_DATA);
#endif
}

bool ToFSensor::is_sensor_data_full() {
    bool res = true;
    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        int head = sensor_data_head[i];
        int tail = sensor_data_tail[i];
        res = res && (head == ((tail + 1) % TOF_NUM_DATA)) && (tail != -1);
        if (!res) break;
    }

    return res;
}
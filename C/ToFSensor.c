#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ToFSensor.h"

ToFSensorData_Common default_data_common = 
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
};

ToFSensorData_Target default_data_target = 
{
  //* Signal returned to the sensor in kcps/spads 
#ifndef VL53L7CX_DISABLE_SIGNAL_PER_SPAD
  //uint32_t signal_per_spad[VL53L7CX_NB_TARGET_PER_ZONE];
  0,
#endif

  //* Sigma of the current distance in mm 
#ifndef VL53L7CX_DISABLE_RANGE_SIGMA_MM
//   uint16_t range_sigma_mm[VL53L7CX_NB_TARGET_PER_ZONE];
  0,
#endif

  //* Measured distance in mm 
#ifndef VL53L7CX_DISABLE_DISTANCE_MM
//   int16_t distance_mm[VL53L7CX_NB_TARGET_PER_ZONE];
  0,
#endif

  //* Estimated reflectance in percent 
#ifndef VL53L7CX_DISABLE_REFLECTANCE_PERCENT
//   uint8_t reflectance[VL53L7CX_NB_TARGET_PER_ZONE];
  0,
#endif

  //* Status indicating the measurement validity (5 & 9 means ranging OK)
#ifndef VL53L7CX_DISABLE_TARGET_STATUS
//   uint8_t target_status[VL53L7CX_NB_TARGET_PER_ZONE];
  0,
#endif
};

// header and tail positions for each sensor and zone
// do we really need to keep heads and tails for all zones? (if all zones are filled simultaneously)
int sensor_data_head[TOF_NUM_SENSORS];
int sensor_data_tail[TOF_NUM_SENSORS];

#if USE_DYNAMIC_ALLOC_FOR_SENSOR_DATA

ToFSensorData_Common ***the_sensor_data_common = NULL;
ToFSensorData_Target ****the_sensor_data_target = NULL;
ToFSensorData ***the_sensor_data = NULL;

void ToF_init_sensor_data() {
    // alloc common sensor data

    the_sensor_data_common = (ToFSensorData_Common***) calloc(TOF_NUM_SENSORS, sizeof(ToFSensorData_Common*));
    if(the_sensor_data_common == NULL) {
        printf("Memory allocation failed.\n");
        exit(1);
    }

    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        the_sensor_data_common[i] = (ToFSensorData_Common**) calloc(TOF_NUM_ZONES, sizeof(ToFSensorData_Common*));
        if(the_sensor_data_common[i] == NULL) {
            printf("Memory allocation failed.\n");
            exit(1);
        }

        for(int j = 0; j < TOF_NUM_ZONES; j++) {
            the_sensor_data_common[i][j] = (ToFSensorData_Common*) calloc(TOF_NUM_DATA, sizeof(ToFSensorData_Common));
            if(the_sensor_data_common[i][j] == NULL) {
                printf("Memory allocation failed.\n");
                exit(1);
            }
        }
    }

    // alloc target sensor data
    the_sensor_data_target = (ToFSensorData_Target****) calloc(TOF_NUM_SENSORS, sizeof(ToFSensorData_Target***));
    if(the_sensor_data_target == NULL) {
        printf("Memory allocation failed.\n");
        exit(1);
    }

    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        the_sensor_data_target[i] = (ToFSensorData_Target***) calloc(TOF_NUM_ZONES, sizeof(ToFSensorData_Target**));
        if(the_sensor_data_target[i] == NULL) {
            printf("Memory allocation failed.\n");
            exit(1);
        }

        for(int j = 0; j < TOF_NUM_ZONES; j++) {
            the_sensor_data_target[i][j] = (ToFSensorData_Target**) calloc(TOF_NUM_TPZ, sizeof(ToFSensorData_Target*));
            if(the_sensor_data_target[i][j] == NULL) {
                printf("Memory allocation failed.\n");
                exit(1);
            }

            for(int k = 0; k < TOF_NUM_TPZ; k++) {
                the_sensor_data_target[i][j][k] = (ToFSensorData_Target*) calloc(TOF_NUM_DATA, sizeof(ToFSensorData_Target));
                if(the_sensor_data_target[i][j][k] == NULL) {
                    printf("Memory allocation failed.\n");
                    exit(1);
                }
            }
        }
    }

    memset(sensor_data_head, 0, TOF_NUM_SENSORS);
    memset(sensor_data_tail, -1, TOF_NUM_SENSORS);

    printf("init_sensor_data successful\n");
}

void ToF_free_sensor_data() {
    // free common sensor data
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

    // free target sensor data
    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        for(int j = 0; j < TOF_NUM_ZONES; j++) {
            for(int k = 0; k < TOF_NUM_TPZ; k++) {
                if(the_sensor_data_target[i][j][k] != NULL)
                    free(the_sensor_data_target[i][j][k]);
            }

            if(the_sensor_data_target[i][j] != NULL)
                free(the_sensor_data_target[i][j]);
        }

        if(the_sensor_data_target[i] != NULL)
            free(the_sensor_data_target[i]);
    }

    if(the_sensor_data_target != NULL)
        free(the_sensor_data_target);

    printf("free_sensor_data successful\n");
}

bool ToF_sensor_connected() {
    return the_sensor_data_common != NULL;
}

#else

ToFSensorData_Common the_sensor_data_common[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA];
ToFSensorData_Target the_sensor_data_target[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_TPZ][TOF_NUM_DATA];
bool sensor_initialized = false;

void ToF_init_sensor_data() {
    memset(the_sensor_data_common, 0, sizeof(ToFSensorData_Common) * TOF_NUM_SENSORS * TOF_NUM_ZONES * TOF_NUM_DATA);
    memset(the_sensor_data_target, 0, sizeof(ToFSensorData_Target) * TOF_NUM_SENSORS * TOF_NUM_ZONES * TOF_NUM_TPZ * TOF_NUM_DATA);

    memset(sensor_data_head, 0, TOF_NUM_SENSORS);
    memset(sensor_data_tail, -1, TOF_NUM_SENSORS);

    sensor_initialized = true
    printf("init_sensor_data successful\n");
}

void free_sensor_data() {
    // no need to do anything here.
    sensor_initialized = false
    printf("free_sensor_data successful\n");
}

bool ToF_sensor_connected() {
    return sensor_initialized;
}

#endif

void ToF_clear_sensor_data() {
    if(the_sensor_data_common == NULL) {
        ToF_init_sensor_data();
    } else {
        for(int i = 0; i < TOF_NUM_SENSORS; i++) {
            for(int j = 0; j < TOF_NUM_ZONES; j++) {
                for(int k = 0; k < TOF_NUM_DATA; k++) {
                    the_sensor_data_common[i][j][k] = default_data_common;
                }
            }
        }

        for(int i = 0; i < TOF_NUM_SENSORS; i++) {
            for(int j = 0; j < TOF_NUM_ZONES; j++) {
                for(int k = 0; k < TOF_NUM_TPZ; k++) {
                    for(int l = 0; l < TOF_NUM_DATA; l++) {
                        the_sensor_data_target[i][j][k][l] = default_data_target;
                    }
                }
            }
        }

        memset(sensor_data_head, 0, TOF_NUM_SENSORS);
        memset(sensor_data_tail, -1, TOF_NUM_SENSORS);
    }

    printf("sensor data clearing successful\n");
}

// always insert a sensor data at the end
void ToF_insert_sensor_data(int iSensor, VL53L7CX_ResultsData* sensor_data) {
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

        for(int j = 0; j < TOF_NUM_TPZ; j++) {
            //* Signal returned to the sensor in kcps/spads 
            #ifndef VL53L7CX_DISABLE_SIGNAL_PER_SPAD
            the_sensor_data_target[iSensor][i][j][next_tail].signal_per_spad = sensor_data->signal_per_spad[i*TOF_NUM_TPZ+j];
            #endif

            //* Sigma of the current distance in mm 
            #ifndef VL53L7CX_DISABLE_RANGE_SIGMA_MM
            the_sensor_data_target[iSensor][i][j][next_tail].range_sigma_mm = sensor_data->range_sigma_mm[i*TOF_NUM_TPZ+j];
            #endif

            //* Measured distance in mm 
            #ifndef VL53L7CX_DISABLE_DISTANCE_MM
            the_sensor_data_target[iSensor][i][j][next_tail].distance_mm = sensor_data->distance_mm[i*TOF_NUM_TPZ+j];
            #endif

            //* Estimated reflectance in percent 
            #ifndef VL53L7CX_DISABLE_REFLECTANCE_PERCENT
            the_sensor_data_target[iSensor][i][j][next_tail].reflectance = sensor_data->reflectance[i*TOF_NUM_TPZ+j];
            #endif

            //* Status indicating the measurement validity (5 & 9 means ranging OK)
            #ifndef VL53L7CX_DISABLE_TARGET_STATUS
            the_sensor_data_target[iSensor][i][j][next_tail].target_status = sensor_data->target_status[i*TOF_NUM_TPZ+j];
            #endif
        }
    }

    // set next head and tail position
    bool sensor_data_full = (next_tail == current_head && current_tail != -1);
    if (sensor_data_full) {
        current_head = (current_head + 1) % TOF_NUM_DATA;
        sensor_data_head[iSensor] = current_head;
    }
    sensor_data_tail[iSensor] = next_tail;
}

void ToF_copy_sensor_data(ToFSensorData_Common dest_common[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA], 
                          ToFSensorData_Target dest_target[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_TPZ][TOF_NUM_DATA]) {
#if USE_DYNAMIC_ALLOC_FOR_SENSOR_DATA
    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        for(int j = 0; j < TOF_NUM_ZONES; j++) {
            for(int k = 0; k < TOF_NUM_DATA; k++) {
                memcpy(dest_common[i][j]+k, the_sensor_data_common[i][j]+k, sizeof(ToFSensorData_Common));
            }
        }
    }
    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        for(int j = 0; j < TOF_NUM_ZONES; j++) {
            for(int k = 0; k < TOF_NUM_TPZ; k++) {
                for(int l = 0; l < TOF_NUM_DATA; l++) {
                    memcpy(dest_target[i][j][k]+l, the_sensor_data_target[i][j][k]+l, sizeof(ToFSensorData_Target));
                }
            }
        }
    }
#else
    memcpy(dest_common, the_sensor_data_common, sizeof(ToFSensorData_Common) * TOF_NUM_SENSORS * TOF_NUM_ZONES * TOF_NUM_DATA);
    memcpy(dest_target, the_sensor_data_target, sizeof(ToFSensorData_Target) * TOF_NUM_SENSORS * TOF_NUM_ZONES * TOF_NUM_TPZ * TOF_NUM_DATA);
#endif
}


bool ToF_is_sensor_data_full() {
    bool res = true;
    for(int i = 0; i < TOF_NUM_SENSORS; i++) {
        int head = sensor_data_head[i];
        int tail = sensor_data_tail[i];
        res = res && (head == ((tail + 1) % TOF_NUM_DATA)) && (tail != -1);
        if (!res) break;
    }

    return res;
}
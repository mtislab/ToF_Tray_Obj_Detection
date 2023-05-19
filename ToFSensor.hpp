#ifndef TOFSENSOR_H
#define TOFSENSOR_H

#include <cstdint>
#include "vl53l7cx_api.h" // for preprocessor definitions

// sensor data 
typedef struct _ToFSensorData {
  //* Ambient noise in kcps/spads 
#ifndef VL53L7CX_DISABLE_AMBIENT_PER_SPAD
  uint32_t ambient_per_spad;
#endif

  //* Number of valid target detected for 1 zone 
#ifndef VL53L7CX_DISABLE_NB_TARGET_DETECTED
  uint8_t nb_target_detected;
#endif

  //* Number of spads enabled for this ranging 
#ifndef VL53L7CX_DISABLE_NB_SPADS_ENABLED
  uint32_t nb_spads_enabled;
#endif

  //* Signal returned to the sensor in kcps/spads 
#ifndef VL53L7CX_DISABLE_SIGNAL_PER_SPAD
  uint32_t signal_per_spad[VL53L7CX_NB_TARGET_PER_ZONE];
#endif

  //* Sigma of the current distance in mm 
#ifndef VL53L7CX_DISABLE_RANGE_SIGMA_MM
  uint16_t range_sigma_mm[VL53L7CX_NB_TARGET_PER_ZONE];
#endif

  //* Measured distance in mm 
#ifndef VL53L7CX_DISABLE_DISTANCE_MM
  int16_t distance_mm[VL53L7CX_NB_TARGET_PER_ZONE];
#endif

  //* Estimated reflectance in percent 
#ifndef VL53L7CX_DISABLE_REFLECTANCE_PERCENT
  uint8_t reflectance[VL53L7CX_NB_TARGET_PER_ZONE];
#endif

  //* Status indicating the measurement validity (5 & 9 means ranging OK)
#ifndef VL53L7CX_DISABLE_TARGET_STATUS
  uint8_t target_status[VL53L7CX_NB_TARGET_PER_ZONE];
#endif
} ToFSensorData;


/* target_status
    Target status Description
    0   Ranging data are not updated
    1   Signal rate too low on SPAD array
    2   Target phase
    3   Sigma estimator too high
    4   Target consistency failed
    5   Range valid
    6   Wrap around not performed (Typically the first range)
    7   Rate consistency failed
    8   Signal rate too low for the current target
    9   Range valid with large pulse (may be due to a merged target)
    10  Range valid, but no target detected at previous range
    11  Measurement consistency failed
    12  Target blurred by another one, due to sharpener
    13  Target detected but inconsistent data. Frequently happens for secondary targets.
    255 No target detected (only if number of target detected is enabled)

    To have consistent data, the user needs to filter invalid target status. 
    To give a confidence rating, a target with status 5 is considered as 100 % valid. 
    A status of 6 or 9 can be considered with a confidence value of 50 %. 
    All other statuses are below 50 % confidence level.
*/

#define TOF_OBSERVE_TIME 2 // length of time to observe ToF data(s)
#define TOF_DATA_RATE 10 // frequency at which data is published(Hz)

// total number of data(intensity and distance) size will be 
// NUM_SENSORS * NUM_ZONES * NUM_DATA * sizeof(ToFSensorData) << isn't it too big?

#define TOF_ZONE_SIZE 8 // ToF sensor zone size
#define TOF_NUM_SENSORS 6 // number of ToF sensors used in object detection
#define TOF_NUM_ZONES (TOF_ZONE_SIZE * TOF_ZONE_SIZE) // 64(8x8) or 16(4x4): number of ToF sensor zones
#define TOF_NUM_DATA (TOF_OBSERVE_TIME * TOF_DATA_RATE)

#define USE_DYNAMIC_ALLOC_FOR_SENSOR_DATA 1

class ToFSensor {
private:
    // header and tail positions for each sensor and zone
    // do we really need to keep heads and tails for all zones? (if all zones are filled simultaneously)
    int sensor_data_head[TOF_NUM_SENSORS];
    int sensor_data_tail[TOF_NUM_SENSORS];

    std::string ToFSensorName;

    // CAUTION: static memory allocation might be necessary for embedded system
#if USE_DYNAMIC_ALLOC_FOR_SENSOR_DATA
    ToFSensorData ***the_sensor_data = NULL; 
#else
    ToFSensorData the_sensor_data[NUM_SENSORS][TOF_NUM_ZONES][NUM_DATA];
#endif

public:
    static ToFSensorData default_data;
    ToFSensor(); // constructor
    ~ToFSensor();

    void init_sensor_data();
    void free_sensor_data();
    void clear_sensor_data();
    void insert_sensor_data(int iSensor, VL53L7CX_ResultsData* sensor_data);
    void copy_sensor_data(ToFSensorData dest[TOF_NUM_SENSORS][TOF_NUM_ZONES][TOF_NUM_DATA]);
    bool is_sensor_data_full();
};

#endif
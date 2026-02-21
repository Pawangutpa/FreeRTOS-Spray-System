#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <Arduino.h>

/*
 * Structure holding raw LiDAR values
 * Received from CAN bus
 */
typedef struct
{
    uint16_t left_distance_cm;
    uint16_t left_signal_strength;

    uint16_t right_distance_cm;
    uint16_t right_signal_strength;

} lidar_data_t;

#endif
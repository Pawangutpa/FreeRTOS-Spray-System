#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <Arduino.h>

/* ==============================
   LIDAR DATA STRUCTURE
   ============================== */
typedef struct
{
    uint16_t left_distance_cm;
    uint16_t left_signal_strength;

    uint16_t right_distance_cm;
    uint16_t right_signal_strength;

} lidar_data_t;

/* ==============================
   PROXI DATA STRUCTURE
   ============================== */
typedef struct
{
    float total_water_liters;

    uint32_t nut_count_left;
    uint32_t nut_count_right;

    float nut_count_average;

} proxi_data_t;

#endif
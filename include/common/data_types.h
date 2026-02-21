#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <stdint.h>

/* ==============================
   LIDAR DATA STRUCTURE
   ============================== */

typedef struct
{
    /* Raw Values */
    uint16_t left_raw_distance_cm;
    uint16_t left_raw_strength;

    uint16_t right_raw_distance_cm;
    uint16_t right_raw_strength;

    /* Filtered Values */
    uint16_t left_distance_cm;
    uint16_t right_distance_cm;

    /* Valid Flags */
    bool left_is_valid;
    bool right_is_valid;

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
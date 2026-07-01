#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"

#include "ms200.h"


void Lidar_Ms200_Init(void);
void Lidar_Ms200_Copy_Data(ms200_data_t *out_data);
uint16_t Lidar_Ms200_Get_Distance(uint16_t angle);
uint16_t Lidar_Ms200_Get_Intensity(uint16_t point);


#ifdef __cplusplus
}
#endif

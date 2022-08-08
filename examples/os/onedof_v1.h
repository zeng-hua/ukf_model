//
// Created by zenghua on 7/28/22.
//

#ifndef UKF_SPRING_DAMPING_SYSTEM_H
#define UKF_SPRING_DAMPING_SYSTEM_H

#include "UKF/Config.h"

#ifdef __cplusplus
extern "C" {

#endif
#define UKF_STATE_DIM 2



void ukf_init(double x0, double v0);



void ukf_sensor_clear() ;

double ukf_get_x();

double ukf_get_v();

void ukf_iterate(double delt);

void ukf_sensor_set_Accelerometer(double acc);

void ukf_sensor_set_Displacementmeter(double dis);

}






#endif //UKF_SPRING_DAMPING_SYSTEM_H

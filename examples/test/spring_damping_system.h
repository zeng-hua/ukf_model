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

    struct sensor_params_t {
        real_t disp;
    };

    struct state_t {
        real_t x_;
        real_t v_;
    };

    struct state_error_t {
        real_t x_;
        real_t v_;

    };


    struct sensor_errors_t {
        real_t disp_bias;
    };

    struct innovation_t {
        real_t accel[3];
        real_t gyro[3];

    };


void init(void);



void set_attitude(real_t w, real_t x, real_t y, real_t z);




}






#endif //UKF_SPRING_DAMPING_SYSTEM_H

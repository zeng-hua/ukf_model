//
// Created by zenghua on 7/28/22.
//

//#include "spring_damping_system.h"
#include <Eigen/Core>
//#include <Eigen/Geometry>
//#include <cmath>
#include "UKF/Types.h"
#include "UKF/Integrator.h"
#include "UKF/StateVector.h"
#include "UKF/MeasurementVector.h"
#include "UKF/Core.h"
#include "UKF/Config.h"

#define MASS (1)
#define DAMP (0.1)
#define K (10)

enum SDY_Keys {
    /* AHRS filter fields. */
    Position,
    Velocity,

    /* Parameter estimation filter fields. */
    Force,

    /* oneDOF measurement vector fields. */
    Acceleration
};

using SDY_StateVector = UKF::StateVector<
        UKF::Field<Position, real_t>,
        UKF::Field<Velocity, real_t>
>;

using SDY_Force = UKF::StateVector<
        UKF::Field<Force, real_t>
>;


/* SDY process model. */
template<> template<>
SDY_StateVector SDY_StateVector::derivative<SDY_Force>(const SDY_Force& input) const {
    SDY_StateVector output;
    /* Position derivative. */
    output.set_field<Position>(get_field<Velocity>());
    /* Velocity derivative. */
    output.set_field<Velocity>(input.get_field<Force>() / MASS - get_field<Velocity>() * DAMP / MASS - get_field<Position>() * K / MASS);
    return output;
};


using SDY_MeasurementVector = UKF::DynamicMeasurementVector<
        UKF::Field<Acceleration, real_t>
>;


template<> template<>
real_t SDY_MeasurementVector::expected_measurement
        <SDY_StateVector, Acceleration, SDY_Force>(
        const SDY_StateVector& state, const SDY_Force& input) {
    return input.get_field<Force>() / MASS - state.get_field<Velocity>() * DAMP / MASS - state.get_field<Position>() * K / MASS;
}

using SDY_Filter = UKF::SquareRootCore<
        SDY_StateVector,
        SDY_MeasurementVector,
        UKF::IntegratorHeun
>;

/* SDY parameter estimation filter process model. */
template <> template <>
SDY_Force SDY_Force::derivative<>() const {
    return SDY_Force::Zero();
}

template <> template <>
real_t SDY_MeasurementVector::expected_measurement
        <SDY_Force, Acceleration, SDY_StateVector>(
        const SDY_Force& state, const SDY_StateVector& input) {
    return state.get_field<Force>() / MASS - input.get_field<Velocity>() * DAMP / MASS - input.get_field<Position>() * K / MASS;
}

/* Just use the Euler integrator since there's no process model. */
using SDY_ParameterEstimationFilter = UKF::SquareRootParameterEstimationCore<
        SDY_Force,
        SDY_MeasurementVector
>;

static SDY_Filter sdys;
static SDY_ParameterEstimationFilter sdys_force;
static SDY_MeasurementVector meas;


int main(){
    /* Initialise state vector and covariance. */
    sdys.state.set_field<Position>(10.0);
    sdys.state.set_field<Velocity>(0.0);
    sdys.root_covariance = SDY_StateVector::CovarianceMatrix::Zero();
    sdys.root_covariance.diagonal() << 1e0, 1e0;
    /* Set measurement noise covariance. */
    sdys.measurement_root_covariance << 0.5;
    /* Set process noise covariance. */
    sdys.process_noise_root_covariance = SDY_StateVector::CovarianceMatrix::Zero();
    sdys.process_noise_root_covariance.diagonal() <<5e-5,5e-5;
    /* Initialise force state vector */
    sdys_force.state.set_field<Force>(1.0);
    /* Initialise force covariance. */
    sdys_force.root_covariance = SDY_Force::CovarianceMatrix::Zero();
    sdys_force.root_covariance.diagonal() << 0.1;
    /* Set measurement noise covariance. */
    sdys_force.measurement_root_covariance << sdys.measurement_root_covariance;
    /* Set process noise covariance. */
    sdys_force.process_noise_root_covariance = SDY_Force::CovarianceMatrix::Zero();
    sdys_force.process_noise_root_covariance.diagonal() <<5e-5;

    /* Iterative process.*/
    int step = 0;
    float dt = 1;
    switch (step++) {
        case 0:
            sdys_force.a_priori_step();
            sdys_force.innovation_step(meas, sdys.state);
            break;
        case 1:
            sdys_force.a_posteriori_step();
            step = 0;
            break;
    }

    sdys.a_priori_step(dt);
    sdys.innovation_step(meas, sdys_force.state);
    sdys.a_posteriori_step();


}


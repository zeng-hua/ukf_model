
#include "onedof_v1.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <cmath>
#include <iostream>
#include "UKF/Types.h"
#include "UKF/Integrator.h"
#include "UKF/StateVector.h"
#include "UKF/MeasurementVector.h"
#include "UKF/Core.h"
#include "UKF/Config.h"

using namespace std;

#define MASS (1)
#define DAMP (0.5)
#define K (10)

enum SDY_Keys {
    /* AHRS filter fields. */
    Position,
    Velocity,

    /* Parameter estimation filter fields. */
    Force,

    /* oneDOF measurement vector fields. */
    Accelerometer,
    Displacementmeter
};

using SDY_StateVector = UKF::StateVector<
        UKF::Field<Position, real_t>,
        UKF::Field<Velocity, real_t>
>;




/* SDY process model. */
template<> template<>
SDY_StateVector SDY_StateVector::derivative<>() const {
    SDY_StateVector output;
    /* Position derivative. */
    output.set_field<Position>(get_field<Velocity>());
    /* Velocity derivative. */
    output.set_field<Velocity>(- get_field<Velocity>() * DAMP / MASS - get_field<Position>() * K / MASS);
    return output;
};


using SDY_MeasurementVector = UKF::DynamicMeasurementVector<
        UKF::Field<Accelerometer, real_t>,
        UKF::Field<Displacementmeter, real_t>
>;


template<> template<>
real_t SDY_MeasurementVector::expected_measurement
        <SDY_StateVector, Accelerometer>(const SDY_StateVector& state) {
    return - state.get_field<Velocity>() * DAMP / MASS - state.get_field<Position>() * K / MASS;
}

template<> template<>
real_t SDY_MeasurementVector::expected_measurement
        <SDY_StateVector, Displacementmeter>(const SDY_StateVector& state) {
    return - state.get_field<Position>();
}

using SDY_Filter = UKF::SquareRootCore<
        SDY_StateVector,
        SDY_MeasurementVector,
        UKF::IntegratorHeun
>;


static SDY_Filter sdys;

static SDY_MeasurementVector meas;


void ukf_init(double x0, double v0) {
    /* Initialise state vector and covariance. */
    sdys.state.set_field<Position>(x0);
    sdys.state.set_field<Velocity>(v0);
    sdys.root_covariance = SDY_StateVector::CovarianceMatrix::Zero();
    sdys.root_covariance.diagonal() << 1e0, 1e0;
    /* Set measurement noise covariance. */
    sdys.measurement_root_covariance << 0.0,0.0;
    /* Set process noise covariance. */
    sdys.process_noise_root_covariance = SDY_StateVector::CovarianceMatrix::Zero();
    sdys.process_noise_root_covariance.diagonal() <<5e-5,5e-5;
}

void ukf_sensor_clear() {
    meas = SDY_MeasurementVector();
}

double ukf_get_x(){
    return sdys.state.get_field<Position>();
}

double ukf_get_v(){
    return sdys.state.get_field<Velocity>();
}

void ukf_iterate(double delt){
    sdys.step(delt, meas);
}

void ukf_sensor_set_Accelerometer(double acc){
    meas.set_field<Accelerometer>(acc);
}

void ukf_sensor_set_Displacementmeter(double dis){
    meas.set_field<Displacementmeter>(dis);
}

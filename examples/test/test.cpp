
//#include "spring_damping_system.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include "UKF/Types.h"
#include "UKF/Integrator.h"
#include "UKF/StateVector.h"
#include "UKF/MeasurementVector.h"
#include "UKF/Core.h"
#include "UKF/Config.h"
#include <iostream>
#include <vector>
#include <string>
#include <fstream>


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
    return state.get_field<Position>();
}

using SDY_Filter = UKF::SquareRootCore<
        SDY_StateVector,
        SDY_MeasurementVector,
        UKF::IntegratorHeun
>;


static SDY_Filter sdys;

static SDY_MeasurementVector meas;
/*
The following functions provide a ctypes-compatible interface for ease of
testing.
*/


int main(){

    vector<double> t,x,v;

    string csvfilename = "/home/zenghua/code/ukf/examples/test/python/onedof.csv";
    vector<string> row;
    string line, word;
    fstream file(csvfilename, ios::in);
    if (file.is_open()) {
        while (getline(file, line)) {
            row.clear();
            stringstream str(line);
            while (getline(str, word, ','))
                row.push_back(word);
            t.push_back(stod(row[0]));
            x.push_back(stod(row[1]));
            v.push_back(stod(row[2]));
        }
    } else {
        cout << "The file " << csvfilename << " cannot be opened!" << endl;
    }


    /* Initialise state vector and covariance. */
    sdys.state.set_field<Position>(0.0);
    sdys.state.set_field<Velocity>(0.0);
    sdys.root_covariance = SDY_StateVector::CovarianceMatrix::Zero();
    sdys.root_covariance.diagonal() << 1e0, 1e0;
    /* Set measurement noise covariance. */
    sdys.measurement_root_covariance << 0.05,0.05;
    /* Set process noise covariance. */
    sdys.process_noise_root_covariance = SDY_StateVector::CovarianceMatrix::Zero();
    sdys.process_noise_root_covariance.diagonal() <<5e-5,5e-5;


//    meas.set_field<Accelerometer>(real_t(2.0));
//    meas.set_field<Displacementmeter>(real_t(2.0));


    int idx = 0;
    double delt = t[2] - t[1];

    vector<double> x_,v_;

    for(auto it = t.begin();it < t.end();it++,idx++){
        meas.set_field<Displacementmeter>(x[idx]);
        sdys.step(delt, meas);

        cout<<"sys x:"<<x[idx]<<"  ukf x:"<<sdys.state.get_field<Position>()<<endl;



        x_.push_back(sdys.state.get_field<Position>());
        v_.push_back(sdys.state.get_field<Velocity>());
    }


    fstream outfile("/home/zenghua/code/ukf/examples/test/python/onedof_cpp.csv", ios::out);
    idx = 0;
    if (outfile.is_open()) {
        for (auto it = t.begin(); it < t.end(); it++, idx++)
            outfile << *it << "," << x_[idx] << "," << v_[idx] << endl;

    }
}
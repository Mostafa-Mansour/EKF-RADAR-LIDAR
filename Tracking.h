//
// Created by mostafa on 21.06.17.
//

#ifndef KF_LIDAR_TRACKING_H
#define KF_LIDAR_TRACKING_H

#include "measurement.h"
#include "ExtendedKalmanFilter.h"
#include "Eigen/Dense"
#include <vector>
using Eigen::VectorXd;
using Eigen::MatrixXd;

class Tracking {
public:
    Tracking();
    virtual ~Tracking();
    std::vector<VectorXd> estimations;
    ExtendedKalmanFilter ekf_;
    void ProcessMeasurement(const MeasurementPackage& measurement);
    void calculateRMSE(const std::vector<Eigen::VectorXd>&,const std::vector<Eigen::VectorXd>& );

private:
    bool is_initialized_;
    long previous_timestamp_;
    float noise_ax;
    float noise_ay;

};


#endif //KF_LIDAR_TRACKING_H

//
// Created by mostafa on 24.06.17.
//

#ifndef EKF_RADAR_EXTENDEDKALMANFILTER_H
#define EKF_RADAR_EXTENDEDKALMANFILTER_H

#include "Eigen/Dense"
using Eigen::VectorXd;
using Eigen::MatrixXd;

class ExtendedKalmanFilter {
public:
    VectorXd X_;
    MatrixXd P_;
    MatrixXd F_;
    MatrixXd Q_;
    MatrixXd H_; //jacobian of the measurement matrix
    MatrixXd R_;

    void predict();
    void updateRadar(const VectorXd&);
    void updateLaser(const VectorXd&);

    MatrixXd getJacobian(const VectorXd&);
    VectorXd car_f(const VectorXd&);
    VectorXd car_h(const VectorXd&);

};


#endif //EKF_RADAR_EXTENDEDKALMANFILTER_H

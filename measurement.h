//
// Created by mostafa on 21.06.17.
//

#ifndef KF_LIDAR_MEASUREMENT_H
#define KF_LIDAR_MEASUREMENT_H

#include "Eigen/Dense"
class MeasurementPackage{
public:
    long timeStamp_;
    enum Sensortype{
        LASER,RADAR
    }sensor_type_;

    Eigen::VectorXd raw_measurements_;
};
#endif //KF_LIDAR_MEASUREMENT_H

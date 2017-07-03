//
// Created by mostafa on 21.06.17.
//

#include <iostream>
#include "Tracking.h"
#include <math.h>
const float pi=std::acos(-1);
Tracking::Tracking() {
    is_initialized_=false;
    previous_timestamp_=0;

    ekf_.X_=VectorXd(4);

    ekf_.P_=MatrixXd(4,4);
    ekf_.P_<<1,0,0,0,
            0,1,0,0,
            0,0,100,0,
            0,0,0,100;

    ekf_.F_=MatrixXd(4,4);
    ekf_.F_<<1,0,1,0,
            0,1,0,1,
            0,0,1,0,
            0,0,0,1;

    noise_ax=9;
    noise_ay=9;
    
}

Tracking::~Tracking() {}

void Tracking::ProcessMeasurement(const MeasurementPackage &measurement) {
    if(!is_initialized_){
        if(measurement.sensor_type_==MeasurementPackage::LASER){
            ekf_.X_<<measurement.raw_measurements_[0],measurement.raw_measurements_[1],0,0;
        }
        else{

            //check the bearing angel and adjust it between -pi and pi
            double angel=measurement.raw_measurements_[1];
            if(angel>pi ){
                float factor=std::ceil(angel/pi);
                angel-=2*pi*factor;
            }
            else if(angel<pi ){
                float factor=std::floor(angel/pi);
                angel-=2*pi*factor;
            }

            // check complete
            ekf_.X_<<measurement.raw_measurements_[0]*std::cos(angel),
                    measurement.raw_measurements_[0]*std::sin(angel),
                    0,0;
        }

        estimations.push_back(ekf_.X_);
        previous_timestamp_=measurement.timeStamp_;
        is_initialized_=true;
        return;
    }
    double dt=(measurement.timeStamp_-previous_timestamp_)/1000000.0;
    previous_timestamp_=measurement.timeStamp_;
    //std::cout<<"dt= "<<dt<<std::endl;
    ekf_.F_<<1,0,dt,0,
            0,1,0,dt,
            0,0,1,0,
            0,0,0,1;
    float dt_2=dt*dt;
    float dt_3=dt_2*dt;
    float dt_4=dt_3*dt;
    ekf_.Q_=MatrixXd(4,4);

    ekf_.Q_<<dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
            0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
            dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
            0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

    //std::cout<<"Q= "<<kf_.Q_<<std::endl;
    ekf_.predict();

    if(measurement.sensor_type_==MeasurementPackage::LASER){
        ekf_.R_=MatrixXd(2,2);
        ekf_.R_<<0.0225,0,
                0,0.0255;
        ekf_.updateLaser(measurement.raw_measurements_);
    }
    else{
        ekf_.R_=MatrixXd(3,3);
        ekf_.R_ << 0.09, 0, 0,
                    0, 0.0009,0,
                    0,0,0.09;
        ekf_.updateRadar(measurement.raw_measurements_);
    }

    estimations.push_back(ekf_.X_);

    std::cout<<"X= "<<ekf_.X_<<std::endl;
    std::cout<<"P= "<<ekf_.P_<<std::endl;
}

void Tracking::calculateRMSE(const std::vector<Eigen::VectorXd> & estimation, const std::vector<Eigen::VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse<<0,0,0,0;
    //std::cout<<"Entered"<<std::endl;
    if(estimation.size()==0 || estimation.size()!=ground_truth.size()){
        std::cout<<"Check your vectors"<<std::endl;
        return;
    }
    VectorXd residual(4);
    for(unsigned int i=0;i<estimation.size();i++){
        residual=estimation[i]-ground_truth[i];
        residual=residual.array()*residual.array();
        rmse+=residual;
    }

    rmse=rmse/estimation.size();
    rmse=rmse.array().sqrt();
    std::cout<<"RMSE is "<<rmse<<std::endl;
}
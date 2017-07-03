//
// Created by mostafa on 24.06.17.
//

#include <iostream>
#include "ExtendedKalmanFilter.h"
//#include "radar.h"
#include <math.h>
const float pi=std::acos(-1);
VectorXd ExtendedKalmanFilter::car_f(const VectorXd & stateVector) {
    return F_*stateVector;
}

VectorXd ExtendedKalmanFilter::car_h(const VectorXd & stateVector) {
    double px=stateVector(0);
    double py=stateVector(1);
    double vx=stateVector(2);
    double vy=stateVector(3);

    double c1=px*px+py*py;
    double c2=std::sqrt(c1);

    VectorXd predictedMeasurement(3);
    predictedMeasurement<<c2,std::atan2(py,px),(px*vx+py*vy)/c2;

    return predictedMeasurement;



}

void ExtendedKalmanFilter::predict() {
    X_=car_f(X_);
    P_=F_*P_*F_.transpose()+Q_;
}

MatrixXd ExtendedKalmanFilter::getJacobian(const VectorXd & stateVector){

    MatrixXd jacobian(3,4);

    double px=stateVector(0);
    double py=stateVector(1);
    double vx=stateVector(2);
    double vy=stateVector(3);

    double c1=px*px+py*py;
    double c2=std::sqrt(c1);
    double c3=c1*c2;

    if(std::fabs(c1)<0.0001){
        std::cout<<"Small values, Dividing by zero"<<std::endl;
        return jacobian;
    }

    jacobian<<(px/c2),   (py/c2),    0,      0,
             -(py/c1),   (px/c1),    0,      0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    return jacobian;


}
void ExtendedKalmanFilter::updateRadar(const VectorXd &z) {


    VectorXd predictedMeasurement(3);
    predictedMeasurement=car_h(X_);

    H_=getJacobian(X_);
    MatrixXd I=MatrixXd::Identity(X_.size(),X_.size());

    VectorXd innov=z-predictedMeasurement;
    //check an angel

    double angel=innov[1];


        std::cout<<"Entered"<<std::endl;
        if (angel > pi) {
            float factor = std::ceil(angel / 2*pi);
            angel -= 2 * pi * factor;
        } else if (angel < -pi) {
            float factor = std::floor(angel / 2*pi);
            angel -= 2 * pi * factor;
        }

    innov[1]=angel;
    // check complete


    MatrixXd S=H_*P_*H_.transpose()+R_;
    MatrixXd K=P_*H_.transpose()*S.inverse();

    X_=X_+K*innov;
    P_=(I-K*H_)*P_;


}

void ExtendedKalmanFilter::updateLaser(const VectorXd &z) {
    H_=MatrixXd(2,4);
    H_<<1,0,0,0,
        0,1,0,0;

    VectorXd innov=z-H_*X_;
    MatrixXd S=H_*P_*H_.transpose()+R_;
    MatrixXd K=P_*H_.transpose()*S.inverse();
    MatrixXd I=MatrixXd::Identity(X_.size(),X_.size());

    X_=X_+K*innov;
    P_=(I-K*H_)*P_;

}


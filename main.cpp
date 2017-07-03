#include <iostream>
#include <fstream>
#include <vector>
#include "Tracking.h"
int main() {
    std::cout << "EKF implementation using RADAR & LIDAR signals for pedestrian detection" << std::endl;

    std::vector<MeasurementPackage> measurementList;
    std::vector<VectorXd> ground_truthList;
    std::vector<VectorXd> estimations;
    MeasurementPackage measurement;
    std::string filePath="/home/mostafa/Desktop/carnd-term2/c++/EKF_RADAR_LIDAR/obj_pose-laser-radar-synthetic-input.txt";
    std::ifstream fileIn(filePath,std::ios::app);
    std::string line;
    std::string sensor;
    double x,y,x_true,y_true,vx_true,vy_true;
    double rho,phi,rhodot;
    long timeStamp;

    VectorXd ground_truth(4);

    if(fileIn.is_open()){
        std::istringstream iss;
        while(std::getline(fileIn,line)){
            iss.str(line);
            iss>>sensor;

            // Check for sensor type
            if(sensor.compare("L")==0){
                measurement.sensor_type_=MeasurementPackage::LASER;
                measurement.raw_measurements_=VectorXd(2);
                iss>>x;
                iss>>y;
                measurement.raw_measurements_<<x,y;
                iss>>timeStamp;
                measurement.timeStamp_=timeStamp;
                iss>>x_true;
                iss>>y_true;
                iss>>vx_true;
                iss>>vy_true;

                ground_truth<<x_true,y_true,vx_true,vy_true;
            }
            else if(sensor.compare("R")==0){
                measurement.sensor_type_=MeasurementPackage::RADAR;
                measurement.raw_measurements_=VectorXd(3);
                iss>>rho;
                iss>>phi;
                iss>>rhodot;
                measurement.raw_measurements_<<rho,phi,rhodot;
                iss>>timeStamp;
                measurement.timeStamp_=timeStamp;
                iss>>x_true;
                iss>>y_true;
                iss>>vx_true;
                iss>>vy_true;

                ground_truth<<x_true,y_true,vx_true,vy_true;
            }
            else{
                std::cout<<"Unknown sensor type"<<std::endl;
                return -1;
            }
            measurementList.push_back(measurement);
            ground_truthList.push_back(ground_truth);

        }

    }
    fileIn.close();
    Tracking track;
    std::cout<<"Test"<<std::endl;
    for(int i=0;i<measurementList.size();i++){
        track.ProcessMeasurement(measurementList[i]);
        //estimations.push_back()

    }
    std::cout<<"Estimations are "<<measurementList.size()<<std::endl;
    std::cout<<"Ground truth are "<<ground_truthList.size()<<std::endl;
    track.calculateRMSE(track.estimations,ground_truthList);
    return 0;
}
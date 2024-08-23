#pragma once
#ifndef EKF_H
#define EKF_H

#include <iostream>
#include <Eigen/Dense>


class EKF {
public :
    // main에서 초기화
    double dt;
    double k;
    double wb;
    /*
        A : State model
        H : Observation model
        Q : covariance of system noise
        R : covariance of observation noise
        P : covariance matrix
        K : Kalman Gain
        P / P_p : covariance / predicted 
        x / x_p : state vector / predicted
        z : observation
    */
    Eigen::MatrixXd A;
    Eigen::MatrixXd B;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
    Eigen::MatrixXd P_p;
    Eigen::MatrixXd K;

    Eigen::VectorXd u;
    Eigen::VectorXd x_p;
    Eigen::VectorXd x;
    Eigen::VectorXd z;

    EKF(int n, int m);

    void Predict();
    void K_gain();
    void Update();
    void Filtering();

};

#endif
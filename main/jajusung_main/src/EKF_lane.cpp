#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include "EKFH.h"

EKF::EKF(int n, int m) 

    : A(n, n), B(n,n), H(m, n), Q(n, n), R(m, m), P(n, n), P_p(n, n), K(n, m), 
      x(n), x_p(n), z(m), u(n) {

    A.setIdentity();
    A(0,1) = u(0) * pow(1 / cos(x(1) + u(1)), 2);

    B.setIdentity();
    // B(0,0) = tan(x(1) + u(1));
    // B(0,1) = u(0) * pow(1 / cos(x(1) + u(1)), 2);
    // B(1,0) = tan(u(1)) / wb;
    // B(1,1) = (u(0) / wb) * pow(1 / cos(u(1)), 2);

    H.setIdentity();

    Q.setIdentity();
    Q << 0.01, 0, 
         0, 0.01;

    R.setIdentity();
    R << 0.1, 0, 
         0, 0.1;

    P.setIdentity();
    P_p.setIdentity();
    K.setIdentity();

    x.setZero();
    x_p.setZero();
    z.setZero();
    u.setZero();

    }

void EKF::Predict() {

    x_p(0) = x(0) + u(0) * tan(x(1) + u(1)) * dt;
    x_p(1) = x(1) + (u(0) / wb) * tan(u(1)) * dt;

    P_p = A * P * A.transpose() + Q;
}

void EKF::K_gain() {
    Eigen::MatrixXd S = H * P_p * H.transpose() + R;
    if(S.determinant()==0)
        std::cerr << "Singular matrix detected in K_gain calculation" << '\n';
    else 
        K = P_p * H.transpose() * S.inverse();
};

void EKF::Update() {
    x = x_p + K * (z - H * x_p);
    P = P_p - K * (K * H * P_p.inverse());
};

void EKF::Filtering() {
    Predict();
    K_gain();
    Update();
};
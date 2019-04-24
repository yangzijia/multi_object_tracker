//
// Created by root on 19-4-24.
//
#include <iostream>
#include "kalmanFilter.h"

using namespace std;

/* Constructor: */
KalmanFilter::KalmanFilter(int _n, int _m) {
    n = _n;
    m = _m;
}

KalmanFilter::KalmanFilter()
{
    n = 2; m = 0;
    A.setIdentity(2, 2); A(0,1) = 0.005;
    H.setIdentity(2, 2); //H(2, 2);  H << 1, 0, 0, 1;
    Q.setIdentity(2, 2); //Q(2, 2);  Q << 1, 0, 0, 1;
    R.setIdentity(2, 2); //R(2, 2);  R << 1, 0, 0, 1;
    X0.setZero(2); //X0(2);  X0 << 0, 0;
    P0 = P0.setIdentity(2, 2) * 3; //P0(2, 2);  P0 << 3, 0, 0, 3;
    I = I.Identity(n, n);
}

/* Set Fixed Matrix */
void KalmanFilter::setFixed(MatrixXf _A, MatrixXf _H, MatrixXf _Q, MatrixXf _R) {
    A = _A;
    H = _H;
    Q = _Q;
    R = _R;
    I = I.Identity(n, n);
}

/* Set Fixed Matrix */
void KalmanFilter::setFixed(MatrixXf _A, MatrixXf _H, MatrixXf _Q, MatrixXf _R, MatrixXf _B) {
    A = _A;
    B = _B;
    H = _H;
    Q = _Q;
    R = _R;
    I = I.Identity(n, n);
}

/* Set Initial Matrix */
void KalmanFilter::setInitial(VectorXf _X0, MatrixXf _P0) {
    X0 = _X0;
    P0 = _P0;
}

/* Do prediction based of physical system (No external input)
*/
void KalmanFilter::predict(void) {
    X = (A * X0);
    P = (A * P0 * A.transpose()) + Q;
}

/* Do prediction based of physical system (with external input)
* U: Control vector
*/
void KalmanFilter::predict(VectorXf U) {
    X = (A * X0) + (B * U);
    P = (A * P0 * A.transpose()) + Q;
}

/* Correct the prediction, using mesaurement
*  Z: mesaure vector
*/
void KalmanFilter::correct(VectorXf Z) {
    K = (P * H.transpose()) * (H * P * H.transpose() + R).inverse();

    X = X + K * (Z - H * X);

    P = (I - K * H) * P;

    X0 = X;
    P0 = P;

}
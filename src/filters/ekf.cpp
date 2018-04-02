#include "ekf.h"

using namespace std;
using namespace arma;

void EKF::model(){}

EKF::EKF(int n, int m){
     this->N = n;
     this->M = m;

     // Initialize matrices
     x = zeros<fmat>(n,1);
     xhat = zeros<fmat>(n,1);
     F = zeros<fmat>(n,n);
     H = zeros<fmat>(m,n);
     P = zeros<fmat>(n,n);
     Phat = zeros<fmat>(n,n);
     K = zeros<fmat>(n,m);

     // Initial misc variables
     R = eye<fmat>(m,m);
     Q = eye<fmat>(n,n);
     In = eye<fmat>(n,n);
     Im = eye<fmat>(m,m);

     g_offset = zeros<fmat>(n,1);
     g_offset(8,0) = 9.81;
     g_offset_m = zeros<fmat>(m,1);
     g_offset_m(4,0) = 9.81;
}

EKF::~EKF(){}

void EKF::init(vector<float> initials){

     // float x,y,z,yaw, v, w;
     // x = initials.at(0);
     // y = initials.at(1);
     // yaw = yaw_offset = initials.at(2);
     // v = initials.at(3);
     // w = initials.at(4);

     // xhat << x << endr
     //      << y << endr
     //      << yaw << endr
     //      << v << endr
     //      << w << endr;

     // // Measurement function simplifies the relationship between state and sensor readings for convenience.
     // // A more realistic measurement function would distinguish between state value and measured value; e.g.:
     // H << 0 << 0 << 0 << 1 << 0 << endr
     //   << 0 << 0 << 0 << 0 << 1 << endr
     //   << 0 << 0 << 0 << 0 << 1 << endr;

     // Q << 0.1 << 0 << 0 << 0 << 0 << endr
     //   << 0 << 0.1 << 0 << 0 << 0 << endr
     //   << 0 << 0 << 0.1 << 0 << 0 << endr
     //   << 0 << 0 << 0 << 0.1 << 0 << endr
     //   << 0 << 0 << 0 << 0 << 0.1 << endr;

     // R << 0.5 << 0 << 0 << endr
     //   << 0 << 0.5 << 0 << endr
     //   << 0 << 0 << 0.1 << endr;

     float _x = initials.at(0);
     float _y = initials.at(1);
     float _z = initials.at(2);
     float _yaw = yaw_offset = initials.at(3);

     xhat.row(0) = _x;
     xhat.row(1) = _y;
     xhat.row(2) = _z;
     xhat.row(14) = _yaw;

     // xhat.t().print("Initial States (transposed): ");

     // Define the constant portions of the state transition Jacobian
     F = eye<fmat>(N,N);
     F(6,9) = 1;    F(7,10) = 1;   F(8,11) = 1;
     F(15,18) = 1;  F(16,19) = 1;  F(17,20) = 1;

     // Define the constant portions of the measurement Jacobian
     H(0,3) = 1;    H(1,4) = 1;    H(2,6) = 1;    H(2,9) = 1;    H(3,7) = 1;
     H(3,10) = 1;   H(4,8) = 1;    H(4,11) = 1;   H(5,12) = 1;   H(6,13) = 1;
     H(7,14) = 1;   H(8,15) = 1;   H(8,18) = 1;   H(9,16) = 1;   H(9,19) = 1;
     H(10,17) = 1;  H(10,20) = 1;  H(11,17) = 1;

}

void EKF::update_measurements(){

    /** TODO: Error-Checking: Size Matches
    * int err = isSizeMatch(readings, rows,cols);
    * if(err == -1) // Rows don't match
    *   cout << "ERROR: Number of rows is not the required " << rows << "!" << endl;
    * else if(err == -2) // Columns don't match
    *   cout << "ERROR: Number of columns is not the required " << rows << "!" << endl;
    */

    // this->z = readings;

}

void EKF::set_R(fmat r){
     R = r;
}

void EKF::set_Q(fmat q){
     Q = q;
}

void EKF::predict(float _dt){

     float x, y, yaw, v, w;
     cur_dt = _dt;

     fmat wk;
     fmat wa = randn<fmat>(3,1);   fmat wg = randn<fmat>(3,1);
     fmat wk9 = zeros<fmat>(9,1);  fmat wk6 = zeros<fmat>(6,1);

     wk = join_vert(wk9,wa); wk = join_vert(wk,wk6);   wk = join_vert(wk, wg);
     // cout << "Size wk: " << size(wk) << endl;
     // wk.t().print("Prediction Noise (trans): ");

     // x = as_scalar(xhat.row(0));
     // y = as_scalar(xhat.row(1));
     // yaw = as_scalar(xhat.row(2));
     // v = as_scalar(xhat.row(3));
     // w = as_scalar(xhat.row(4));

     // // Predict estimated states
     // xhat << x + v*_dt*cos(yaw) << endr
     //      << y + v*_dt*sin(yaw) << endr
     //      << yaw + w*_dt << endr
     //      << v << endr
     //      << w << endr;

     // F << 1 << 0 << -v*_dt*sin(yaw) << _dt*cos(yaw) << 0 << endr
     //   << 0 << 1 << -v*_dt*cos(yaw) << _dt*sin(yaw) << 0 << endr
     //   << 0 << 0 << 1 << 0 << _dt << endr
     //   << 0 << 0 << 0 << 1 << 0 << endr
     //   << 0 << 0 << 0 << 0 << 1 << endr;

     // Update process model Jacobian
     F(0,3) = _dt;       F(0,6) = (_dt*_dt) / 2;
     F(1,4) = _dt;       F(1,7) = (_dt*_dt) / 2;
     F(2,5) = _dt;       F(2,8) = (_dt*_dt) / 2;
     F(3,6) = _dt;       F(4,7) = _dt;           F(5,8) = _dt;
     F(12,15) = _dt;     F(13,16) = _dt;         F(14,17) = _dt;

     // Predict new estimated states
     xhat = F * xhat + wk;
     // cout << "Size xhat (prediction): " << size(xhat) << endl;

     // Update prediction error
     Phat = F * Phat * trans(F) + Q;
     // cout << "Size P (prediction): " << size(Phat) << endl;
}

void EKF::update(fmat data){

     fmat S,K,residual, zPred;
     float prev_yaw, prev_yaw_rate, xdot, _dt;
     _dt = cur_dt;

     // cout << "Update Step: " << steps << endl;
     // xhat.t().print("    X Post-Prediction (transposed): ");
     // tmpP.print("P Post-Predict:");


     // xdot = as_scalar(xhat.row(3));
     // prev_yaw = as_scalar(xhat.row(11));
     // prev_yaw_rate = as_scalar(xhat.row(14));

     // float a = prev_yaw + prev_yaw_rate * _dt;

     // // Update Measurement Jacobian: Encoder
     // H(0,3) = _dt*cos(a);     H(0,11) = -xdot*_dt*sin(a);   H(0,14) = -xdot*_dt*_dt*sin(a);
     // H(1,3) = _dt*sin(a);     H(1,11) = xdot*_dt*cos(a);    H(1,14) = xdot*_dt*_dt*cos(a);
     // H(2,14) = _dt;
     //
     // // Update Measurement Jacobian: IMU
     // H(5,3) = _dt;       H(5,6) = (_dt*_dt) / 2;
     // H(6,4) = _dt;       H(6,7) = (_dt*_dt) / 2;
     // H(7,5) = _dt;       H(7,8) = (_dt*_dt) / 2;
     // H(8,6) = _dt;       H(9,7) = _dt;            H(10,8) = _dt;
     // H(14,12) = _dt;     H(15,13) = _dt;          H(16,14) = _dt;

     // Innovation Stage
     zPred = H * xhat;
     residual = data - zPred;
     // residual.t().print("     Residual (transposed):");

     // Update Innovation Covariance
     S = H * Phat * trans(H) + R;

     // Update Kalman Gain
     K = Phat * trans(H) * inv(S);


     xhat = xhat + K * residual;
     Phat = (In - K * H) * Phat;
     // Phat = P - K * S * trans(K);

     // xhat.t().print("    X Update (transposed):");
     // Phat.print("P Update:");

     steps++;
}

void EKF::print_internals(){
     F.print("F: ");
     H.print("H: ");
     R.t().print("R (transposed): ");
     Q.t().print("Q (transposed): ");
}

void EKF::print_stats(){
     cout << "xhat Size: " << size(xhat) << endl;
     cout << "F Size: " << size(F) << endl;
     cout << "H Size: " << size(H) << endl;
     cout << "R Size: " << size(R) << endl;
     cout << "Q Size: " << size(Q) << endl;
}

#include "ekf.h"

using namespace std;
using namespace arma;

// void EKF::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]){}
void EKF::model(){}

EKF::EKF(int n, int m){
     this->N = n;
     this->M = m;

    // Initialize null matrices
    x = zeros<fmat>(n,1);
    xhat = zeros<fmat>(n,1);
    F = zeros<fmat>(n,n);
    H = zeros<fmat>(m,n);
    P = zeros<fmat>(n,n);
    Q = zeros<fmat>(n,n);
    R = zeros<fmat>(m,m);
    K = zeros<fmat>(n,m);

    // Initial misc variables
    I = eye<fmat>(n,n);
}

EKF::~EKF(){}

void EKF::init(vector<float> initials){
     float x,y,yaw, v, w;

     x = initials.at(0);
     y = initials.at(1);
     yaw = yaw_offset = initials.at(2);
     v = initials.at(3);
     w = initials.at(4);

     xhat << x << endr
          << y << endr
          << yaw << endr
          << v << endr
          << w << endr;

     // Measurement function simplifies the relationship between state and sensor readings for convenience.
     // A more realistic measurement function would distinguish between state value and measured value; e.g.:
     H << 0 << 0 << 0 << 1 << 0 << endr
       << 0 << 0 << 0 << 0 << 1 << endr
       << 0 << 0 << 0 << 0 << 1 << endr;

     Q << 0.1 << 0 << 0 << 0 << 0 << endr
       << 0 << 0.1 << 0 << 0 << 0 << endr
       << 0 << 0 << 0.1 << 0 << 0 << endr
       << 0 << 0 << 0 << 0.1 << 0 << endr
       << 0 << 0 << 0 << 0 << 0.1 << endr;

     R << 0.1 << 0 << 0 << endr
       << 0 << 0.3 << 0 << endr
       << 0 << 0 << 0.02 << endr;

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

void EKF::predict(float _dt){
     // // Process model is f(x) = x
     // fx[0] = x + v*dt*cos(yaw);
     // fx[1] = y + v*dt*sin(yaw);
     // fx[2] = yaw + w*dt;
     // fx[3] = v;
     // fx[4] = w;

     float x, y, yaw, v, w;
     x = as_scalar(xhat.row(0));
     y = as_scalar(xhat.row(1));
     yaw = as_scalar(xhat.row(2));
     v = as_scalar(xhat.row(3));
     w = as_scalar(xhat.row(4));

     // Predict estimated states
     xhat << x + v*_dt*cos(yaw) << endr
          << y + v*_dt*sin(yaw) << endr
          << yaw + w*_dt << endr
          << v << endr
          << w << endr;

     // Update process model Jacobian
     F << 1 << 0 << -v*_dt*sin(yaw) << _dt*cos(yaw) << 0 << endr
       << 0 << 1 << -v*_dt*cos(yaw) << _dt*sin(yaw) << 0 << endr
       << 0 << 0 << 1 << 0 << _dt << endr
       << 0 << 0 << 0 << 1 << 0 << endr
       << 0 << 0 << 0 << 0 << 1 << endr;

     // Update prediction error
     Phat = F * I * trans(F) + Q;

}

void EKF::update(fmat data){

     fmat S;     						// Innovation Covariance
     fmat K;     						// Kalman Gain
     fmat residual, zPred;

	// tmpX.print("X Post-Predict:");
	// tmpP.print("P Post-Predict:");

     // Innovation Stage
     zPred = H * xhat;
     residual = data - zPred;

     // Update Stage
     S = H * Phat * trans(H) + R;
     K = Phat * trans(H) * inv(S); // Probably more correct version, unsure at this point

     // residual.print("Residual:");

     xhat = xhat + K * residual;
     Phat = (I - K * H) * Phat;
     // Phat = P - K * S * trans(K);

     xhat.print("X Update:");
     // Phat.print("P Update:");

     steps++;
}

// void EKF::model(float fx[Nsta], float F[Nsta][Nsta], float hx[Mobs], float H[Mobs][Nsta]){
//
//      float x, y, yaw, v, w;
//      x = this->x[0];
//      y = this->x[1];
//      yaw = this->x[2];
//      v = this->x[3];
//      w = this->x[4];
//
//      // Process model is f(x) = x
//      fx[0] = x + v*dt*cos(yaw);
//      fx[1] = y + v*dt*sin(yaw);
//      fx[2] = yaw + w*dt;
//      fx[3] = v;
//      fx[4] = w;
//
//      // So process model Jacobian is identity matrix
//      F << 1 << 0 << -v*dt*sin(yaw) << dt*cos(yaw) << 0 << endr
//        << 0 << 1 << -v*dt*cos(yaw) << dt*sin(yaw) << 0 << endr
//        << 0 << 0 << 1 << 0 << dt << endr
//        << 0 << 0 << 0 << 1 << 0 << endr
//        << 0 << 0 << 0 << 0 << 1 << endr;
//
//      // Measurement function simplifies the relationship between state and sensor readings for convenience.
//      // A more realistic measurement function would distinguish between state value and measured value; e.g.:
//      H << 0 << 0 << 0 << 1 << 0 << endr
//        << 0 << 0 << 0 << 0 << 1 << endr
//        << 0 << 0 << 0 << 0 << 1 << endr;
//        // <<  <<  <<  <<  <<  << endr
//
// }

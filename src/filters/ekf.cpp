#include "ekf.h"

using namespace arma;

// void EKF::model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]){}
void EKF::model(){}

EKF::EKF(int n, int m){
     this->Nsta = n;
     this->Mobs = m;
    // // Extract Required number of states and observations from Measurement Jacobian
    // int tmpN = H.n_cols;
    // int tmpM = H.n_rows;
    //
    // // Store class private variables
    // this->n = tmpN;
    // this->m = tmpM;
    // this->dt = dt_init;
    // this->steps = 0;
    // this->I = eye<fmat>(tmpN,tmpN);
    //
    // // Store Initial State Parameters
    // this->params.x = x_init;
    // this->params.P = Pinit;
    // this->params.F = F;
    // this->params.H = H;
    // this->params.Q = Q;
    // this->params.R = R;
    //
    // // Initialize Additional Parameters
    // this->z = zeros<fmat>(tmpM,1);
    // this->params.K = zeros<fmat>(tmpN,tmpM);
    //
    // // Initialize Prediction parameters
    // this->params_pred = this->params;

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


void EKF::predict(){

//      fmat tmpX,tmpP,tmpF,tmpH,tmpQ,tmpR,tmpZ;
//
//      fmat xhat = this->params_pred.x;  	// Predicted State
//      fmat Phat = this->params_pred.P;  	// Predicted State Covariance
//      fmat S;     						// Innovation Covariance
//      fmat K;     						// Kalman Gain
//      fmat residual, zPred, zObs;
//      int step = this->steps;
//
//      tmpF = this->params_pred.F;
//      tmpH = this->params_pred.H;
//      tmpQ = this->params_pred.Q;
//      tmpR = this->params_pred.R;
//      zObs = this->z;
//
// #ifdef DEBUG_EKF
//      xhat.print("X Pre-Predict:");
//      Phat.print("P Pre-Predict:");
// #endif
//
//      float trueX = sin((float) step * M_DEG2RAD);
//      // Prediction Stage
//      tmpX = tmpF * xhat;
//      tmpP = tmpF * I * trans(tmpF) + tmpQ;
//
// #ifdef DEBUG_EKF
// 	tmpX.print("X Post-Predict:");
// 	tmpP.print("P Post-Predict:");
// #endif
//
//      // Innovation Stage
//      zPred = sin((float) step * M_DEG2RAD);
//      // zPred = as_scalar(tmpX.row(1));
//      residual = zObs - zPred;
//
//      // Update Stage
//      S = tmpH * tmpP * trans(tmpH) + tmpR;
//      // K = tmpP * trans(tmpH) / S; // Probably more correct version, unsure at this point
//      K = tmpP * trans(tmpH) / as_scalar(S);
//
// #ifdef DEBUG_EKF
//      residual.print("Residual:");
// #endif
//
//      xhat = xhat + K * residual;
//      // Phat = tmpP - K * S * trans(K);
//      Phat = (this->I - K * tmpH) * Phat;
//
//      this->params_pred.x = xhat;
//      this->params_pred.P = Phat;
//
// #ifdef DEBUG_EKF
//      xhat.print("X Update:");
//      Phat.print("P Update:");
// #endif
//
//      this->steps++;

}

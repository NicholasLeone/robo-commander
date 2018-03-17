#ifndef EKF_H_
#define EKF_H_

#include <armadillo>
#include "base/definitions.h"

using namespace arma;

typedef struct EKF_PARAMS{
    fmat x;         // State Matrix                         [N x 1]
    fmat F;         // Process Model Jacobian               [N x N]
    fmat H;         // Measurement Model Jacobian           [M x N]
    fmat P;         // Predicted State Error Covariance     [N x N]
    fmat Q;         // Process Noise Covariance             [N x N]
    fmat R;         // Measurement Error Covariance         [M x M]
    fmat K;         // Kalman gain                          [N x M]
} EKF_PARAMS;


class EKF {

    private:



    public:

        int n;          // Number of state variables
        int m;          // Number of observed variables
        float dt;       // Change in time from last measurement period
        int steps;      // Number of updates since initialization

        EKF_PARAMS params;      // Initial Parameters
        EKF_PARAMS params_pred; // Current Prediction's Parameters

        fmat I;
        fmat z;         // Observation Matrix                   [M x 1]


        /** Initializes an EKF object */
        EKF(fmat x_init, fmat Pinit, fmat F, fmat H, fmat Q, fmat R, float dt_init);

        /** Deallocates memory for an EKF object */
        //~EKF();

        /**
       * Updates the locally stored observations for use in next prediction step
       *
       *    @param z: the currently observed sensor readings
       */
        void updateObservations(fmat readings);


        /**
       * Updates predicted states and covariances.
       */
        void predict();


};


#endif // EKF_H_

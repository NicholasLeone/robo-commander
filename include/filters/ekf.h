#ifndef EKF_H_
#define EKF_H_

#include <stdio.h>
#include <stdlib.h>
#include <armadillo>
#include "../../include/params.h"

using namespace arma;


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

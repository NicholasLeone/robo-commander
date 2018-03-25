#ifndef EKF_H_
#define EKF_H_

// #define Nsta 2     // # of states (REQUIRED BY TINYEKF)
// #define Mobs 3     // # of measurements (REQUIRED BY TINYEKF)

#include <armadillo>
#include "base/definitions.h"
// #include <TinyEKF.h>

using namespace std;
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

class EKF{
protected:

     // virtual void model(double fx[Nsta], double F[Nsta][Nsta], double hx[Mobs], double H[Mobs][Nsta]) = 0;

private:
     int N;          // Number of state variables
     int M;          // Number of observed variables

     fmat x;         // State Matrix                         [N x 1]
     fmat F;         // Process Model Jacobian               [N x N]
     fmat H;         // Measurement Model Jacobian           [M x N]
     fmat P;         // Predicted State Error Covariance     [N x N]
     fmat Q;         // Process Noise Covariance             [N x N]
     fmat R;         // Measurement Error Covariance         [M x M]
     fmat K;         // Kalman gain                          [N x M]
     fmat I;         // Identity Matrix                      [N x N]
public:

     float dt;       // Change in time from last measurement period
     int steps = 0;  // Number of updates since initialization
     float yaw_offset;
     fmat xhat;
     fmat Phat;
     fmat z;         // Observation Matrix                   [M x 1]

     EKF(int n, int m);
     ~EKF();

     void init(vector<float> initials);

     /**
     * Updates the locally stored observations for use in next prediction step
     *
     *    @param z: the currently observed sensor readings
     */
     void update_measurements();


     /**
     * Updates predicted states and covariances.
     */
     void predict(float _dt);

     /**
     * Updates predicted states and covariances.
     */
     void update(fmat data);

     /**
     * Set the model transitions
     */
     void model();
};


#endif // EKF_H_

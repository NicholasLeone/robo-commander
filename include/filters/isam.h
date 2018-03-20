#ifndef ISAM_H_
#define ISAM_H_

#include "base/definitions.h"

// Both relative poses and recovered trajectory poses will be stored as Pose2 objects
#include <gtsam/geometry/Pose2.h>
// Range observations of the other agents will be stored as Point2 (x, y).
#include <gtsam/geometry/Point2.h>

// Each variable in the system (poses and landmarks) must be identified with a unique key.
// We can either use simple integer keys (1, 2, 3, ...) or symbols (X1, X2, L1).
// Here we will use Symbols
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// We want to use iSAM2 to solve the range-SLAM problem incrementally
#include <gtsam/nonlinear/ISAM2.h>

// iSAM2 requires as input a set of new factors to be added stored in a factor graph,
// and initial guesses for any new variables used in the added factors
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics SLAM problems.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>

using namespace gtsam;
namespace NM = gtsam::noiseModel;

class iSAM2{
private:

     /**Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps to maintain proper linearization
     * and efficient variable ordering, iSAM2 performs partial relinearization/reordering at each step. A parameter
     * structure is available that allows the user to set various properties, such as the relinearization threshold
     * and type of linear solver. For this example, we we set the relinearization threshold small so the iSAM2 result
     * will approach the batch result.*/
     ISAM2Params parameters;
     ISAM2 isam;

     // Flags
	bool flag_odom_initialized;
	bool fresh_odom;

     // Set Noise parameters
	float priorSig = 0.0001;     // Std dev of priors (meters)
	float sigmaR = 0.01;         // Std dev of imu measurements

     // State Variables
     float initX[3] = {1.709, -2.1478, 1.8315};
     float oldX[3];

     // Performance Analysis Values
     float SSE = 0;
     float MSE = 0;

     // Create a Factor Graph and Values to hold the new data
	NonlinearFactorGraph graph;
	Values initialEstimate;
	Values currentEstimate;
	Values estimated_odom;
	Values odomValues;

     // Define Noise Models for the various sensors (for iSAM2)
	NM::Diagonal::shared_ptr priorNoise;
	NM::Diagonal::shared_ptr odomNoise;
	NM::Isotropic::shared_ptr imuNoise;

     // Define the odometry inputs as a Pose2
	Pose2 initPose, prevPose;
	Pose2 odometry;

public:

     int n;                   // Number of state variables
     int m;                   // Number of observed variables
     float dt;                // Change in time from last measurement period
     int num_updates = 0;     // Number of updates since initialization
     int num_odom = 0;        // Number of odometry samples since initialization
     int num_imu = 0;         // Number of imu samples since initialization


     iSAM2();
     ~iSAM2();


     /**
     * Updates the locally stored observations for use in next prediction step
     *
     *    @param deltas: the currently observed sensor readings
     */
     void update_odometry(float deltas[3]);

     /**
     * Updates the locally stored observations for use in next prediction step
     *
     *    @param z: the currently observed sensor readings
     */
     void update_imu();

     /**
     * Updates the locally stored observations for use in next prediction step
     *
     *    @param z: the currently observed sensor readings
     */
     void update();

     /**
     * Updates predicted states and covariances.
     */
     void predict();

     /**
     * Set the model transitions
     */
     void model();
};


#endif // ISAM_H_

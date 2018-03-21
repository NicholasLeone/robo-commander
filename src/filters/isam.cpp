#include "isam.h"

using namespace std;
using namespace gtsam;
namespace NM = gtsam::noiseModel;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

iSAM2::iSAM2(){

     // Initialize variables
     oldX[0] = initX[0]; oldX[1] = initX[1]; oldX[2] = initX[2];
     flag_odom_initialized = false;
     fresh_odom = false;

     // Debugging
     // printf("[INITIALS]:\r\n  X, Y, Z: %.4f, %.4f, %.4f\r\n",initX[0], initX[1], initX[2]);
     // printf("[PREVIOUS]:\r\n  X, Y, Z: %.4f, %.4f, %.4f\r\n",oldX[0], oldX[1], oldX[2]);

     /** Define our iSAM2 parameters */
     parameters.relinearizeThreshold = 0.01;
     parameters.relinearizeSkip = 1;
 	ISAM2 isam(parameters);

	// Initialize noise models to be used by GTSAM libraries
	Vector priorSigmas = Vector3(priorSig,priorSig,priorSig);
	Vector odomSigmas = Vector3(0.5,0.5,0.5); // Randomly selected value for now TODO->should be based on model

     priorNoise = NM::Diagonal::Sigmas(priorSigmas);  //prior
     odomNoise = NM::Diagonal::Sigmas(odomSigmas);  //odom
	imuNoise = NM::Isotropic::Sigma(1, sigmaR);      // non-robust

     /**------------- IMU Section --------------*/
     // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
     Rot3 prior_rotation = Rot3::Quaternion(1, 0, 0, 0);
     Point3 prior_point(0, 0, 0);
     Pose3 prior_pose(prior_rotation, prior_point);
     Vector3 prior_velocity(0, 0, 0);

     imuBias::ConstantBias prior_imu_bias; // assume zero initial bias

     // Add prior poses to the factor graph
	initPose = prevPose = Pose2(initX[0], initX[1], initX[2]);
	graph.push_back(PriorFactor<Pose2>(Symbol('X', 0), initPose, priorNoise));

     int correction_count = 0;
     initial_values.insert(X(correction_count), prior_pose);
     initial_values.insert(V(correction_count), prior_velocity);
     initial_values.insert(B(correction_count), prior_imu_bias);

     // Assemble prior noise model and add it the graph.
     noiseModel::Diagonal::shared_ptr pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
     noiseModel::Diagonal::shared_ptr velocity_noise_model = noiseModel::Isotropic::Sigma(3,0.1); // m/s
     noiseModel::Diagonal::shared_ptr bias_noise_model = noiseModel::Isotropic::Sigma(6,1e-3);

     graph->add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
     graph->add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
     graph->add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));

     float accel_noise_sigma = 0.0003924;
     float gyro_noise_sigma = 0.000205689024915;
     float accel_bias_rw_sigma = 0.004905;
     float gyro_bias_rw_sigma = 0.000001454441043;
     Matrix33 measured_acc_cov = Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
     Matrix33 measured_omega_cov = Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
     Matrix33 integration_error_cov = Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
     Matrix33 bias_acc_cov = Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
     Matrix33 bias_omega_cov = Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
     Matrix66 bias_acc_omega_int = Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration

     boost::shared_ptr<PreintegratedCombinedMeasurements::Params> p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
     // PreintegrationBase params:
     p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
     p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
     // should be using 2nd order integration
     // PreintegratedRotation params:
     p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
     // PreintegrationCombinedMeasurements params:
     p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
     p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
     p->biasAccOmegaInt = bias_acc_omega_int;

     #ifdef USE_COMBINED
          imu_preintegrated_ = new PreintegratedCombinedMeasurements(p, prior_imu_bias);
     #else
          imu_preintegrated_ = new PreintegratedImuMeasurements(p, prior_imu_bias);
     #endif

     // Store previous state for the imu integration and the latest predicted outcome.
     NavState prev_state(prior_pose, prior_velocity);
     NavState prop_state = prev_state;
     imuBias::ConstantBias prev_bias = prior_imu_bias;

     // Keep track of the total error over the entire run for a simple performance metric.
     float current_position_error = 0.0, current_orientation_error = 0.0;


     // Store Initial poses into the 'Values' containers for later analysis
	initialEstimate.insert(Symbol('X', 0), initPose);
	// Print out stored variables for debugging
	initialEstimate.print("Initial Pose: ");

     /** Initialize Misc Variables */

}

iSAM2::~iSAM2(){}

void iSAM2::update_odometry(float dist_traveled, float dyaw){
     num_odom_updates++;

     // // Setup the noise model
     // Vector odoCov = Vector3(covX,covY,covYaw);
     // odomNoise = NM::Diagonal::Variances(odoCov);

	// BetweenFactor was found to work when only using the change in pose relative to the body frame
	odometry = Pose2(dist_traveled, 0, dyaw);

	// Store important variables for later analysis
	odomValues.insert(num_odom_updates, odometry);

	// Ensure we load the BetweenFactor in the factor graph with proper linking of keys
    	if(flag_odom_initialized) {
        	graph.push_back(BetweenFactor<Pose2>(Symbol('X',num_odom_updates-1), Symbol('X',num_odom_updates), odometry, odomNoise));
	} else {
        	graph.push_back(BetweenFactor<Pose2>(Symbol('X',0), Symbol('X',num_odom_updates), odometry, odomNoise));
	}

	/** NOTE:
		Using simulated control inputs from the changes in the estimated pose
	recieved from the dead-reckoned path easily obtained from Turtlebot,
	predict our new pose estimate from our last known pose
	*/
    	Pose2 predPose = prevPose.compose(odometry);

	// Add this predicted pose to our initial estimate
	initialEstimate.insert(Symbol('X', num_odom_updates), predPose);

	// Update Global variables
	prevPose = predPose;

	flag_odom_initialized = true;

}

void iSAM2::update_imu(){

	// Update Global counter
  	num_imu_updates++;

	// Add IMU factor to the factor graph
  	// graph.push_back(RangeFactor<Pose2, Pose2>(Symbol('A', old_odom_counter), Symbol('B', old_odom_counter_2), range_in, rangeNoise));

	// Perform one iSAM2 update step and get the current estimated poses
	isam.update(graph, initialEstimate);
	currentEstimate = isam.calculateEstimate();

	// Print out estimate for debug
	// currentEstimate.print("Current Estimate");

	// Retrieve most recent estimate of each agent
	Pose2 curEstimate = currentEstimate.at<Pose2>(Symbol('X', num_updates));

	// Reset factor graph and initial estimate values for next update step
	graph.resize(0);
	initialEstimate.clear();


}


void iSAM2::update(){
     num_updates++;

     // gttic_(update);
	// Update iSAM with the new factors
	isam.update(graph, initialEstimate);

     // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
	// If accuracy is desired at the expense of time, update(*) can be called additional times
	// to perform multiple optimizer iterations every step.
	isam.update();
	// gttoc_(update);

	// gttic_(calculateEstimate);
	currentEstimate = isam.calculateEstimate();
	// gttoc_(calculateEstimate);

     // Print timings
     // tictoc_print_();

     // cout << "****************************************************" << endl;
	currentEstimate.print("Current estimate: ");
	Pose2 curEstimate = currentEstimate.at<Pose2>(Symbol('X', num_odom_updates));
     curEstimate.print("Latest Estimate: ");

	// Clear the factor graph and values for the next iteration
	graph.resize(0);
	initialEstimate.clear();
}

void iSAM2::predict(){}
void iSAM2::model(){}

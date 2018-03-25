#include "isam.h"

#define USE_COMBINED

using namespace std;
using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

iSAM2::iSAM2(){}

iSAM2::~iSAM2(){}


void iSAM2::init(vector<float> params){

     /** Define our iSAM2 parameters */
     parameters.relinearizeThreshold = 0.01;
     parameters.relinearizeSkip = 1;
     ISAM2 isam(parameters);

     flag_odom_initialized = false;
     fresh_odom = false;

     float x,y,z,qx,qy,qz,qw;
     x = params.at(0);
     y = params.at(1);
     z = params.at(2);
     qx = params.at(3);
     qy = params.at(4);
     qz = params.at(5);
     qw = params.at(6);

     // Initialize variables
     oldX[0] = initX[0] = x;
     oldX[1] = initX[1] = y;
     oldX[2] = initX[2] = z;

     oldX[3] = initX[3] = qw;
     oldX[4] = initX[4] = qz;
     oldX[5] = initX[5] = qy;
     oldX[6] = initX[6] = qx;

     // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
     Rot3 prior_rotation = Rot3::Quaternion(qw, qz, qy, qx);
     Point3 prior_point(x,y,z);
     Pose3 prior_pose(prior_rotation, prior_point);
     Vector3 prior_velocity(0, 0, 0);
     initPose = prevPose = Pose2(initX[0], initX[1], initX[2]);

	// Initialize noise models to be used by GTSAM libraries
	Vector priorSigmas = Vector3(priorSig,priorSig,priorSig);
	Vector odomSigmas = Vector3(0.5,0.5,0.5); // Randomly selected value for now TODO->should be based on model

     priorNoise = NM::Diagonal::Sigmas(priorSigmas);  //prior
     odomNoise = NM::Diagonal::Sigmas(odomSigmas);  //odom
	imuNoise = NM::Isotropic::Sigma(1, sigmaR);      // non-robust
     pose_noise_model = NM::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
     velocity_noise_model = NM::Isotropic::Sigma(3,0.1); // m/s
     bias_noise_model = NM::Isotropic::Sigma(6,1e-3);
     correction_noise = NM::Isotropic::Sigma(3,1.0);

     // Store Initial poses into the 'Values' containers for later analysis
     // initialEstimate.insert(Symbol('X', 0), initPose);
     initialEstimate.insert(X(num_imu_updates), prior_pose);
     initialEstimate.insert(V(num_imu_updates), prior_velocity);
     initialEstimate.insert(B(num_imu_updates), prior_imu_bias);

     // Add prior poses to the factor graph
     // graph.push_back(PriorFactor<Pose2>(Symbol('X', 0), initPose, priorNoise));
     graph.add(PriorFactor<Pose3>(X(num_imu_updates), prior_pose, pose_noise_model));
     graph.add(PriorFactor<Vector3>(V(num_imu_updates), prior_velocity,velocity_noise_model));
     graph.add(PriorFactor<imuBias::ConstantBias>(B(num_imu_updates), prior_imu_bias,bias_noise_model));

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

     p = PreintegratedCombinedMeasurements::Params::MakeSharedD(0.0);
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
     pred_state = prev_state;
     prev_bias = prior_imu_bias;

     // Keep track of the total error over the entire run for a simple performance metric.
     current_position_error = current_orientation_error = 0.0;

	// Print out stored variables for debugging
	initialEstimate.print("Initial Pose: ");

     /** Initialize Misc Variables */
}

void iSAM2::update_odometry(float dist_traveled, float dyaw){
     num_odom_updates++;

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

void iSAM2::update_imu(vector<float> data){

     float ax, ay, az, gx, gy, gz, mx, my, mz, qx, qy, qz, qw, dt;

	// Update Global counter
  	num_imu_updates++;

     dt = data.at(0);
     ax = data.at(1);
     ay = data.at(2);
     az = data.at(3);
     gx = data.at(4);
     gy = data.at(5);
     gz = data.at(6);

     // Adding the IMU preintegration.
     imu_preintegrated_->integrateMeasurement(Vector3(ax, ay, az), Vector3(gx, gy, gz), dt);

#ifdef USE_COMBINED
     PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated_);
     CombinedImuFactor imu_factor(X(num_imu_updates-1), V(num_imu_updates-1),
                                  X(num_imu_updates  ), V(num_imu_updates  ),
                                  B(num_imu_updates-1), B(num_imu_updates  ),
                                  *preint_imu_combined);
     graph.add(imu_factor);
#else
     PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
     ImuFactor imu_factor(X(num_imu_updates-1), V(num_imu_updates-1),
                          X(num_imu_updates  ), V(num_imu_updates  ),
                          B(num_imu_updates-1),
                          *preint_imu);
     graph.add(imu_factor);
     imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
     graph.add(BetweenFactor<imuBias::ConstantBias>(B(num_imu_updates-1),
                                                     B(num_imu_updates  ),
                                                     zero_bias, bias_noise_model));
#endif

     pred_state = imu_preintegrated_->predict(prev_state, prev_bias);
     initialEstimate.insert(X(num_imu_updates), pred_state.pose());
     initialEstimate.insert(V(num_imu_updates), pred_state.v());
     initialEstimate.insert(B(num_imu_updates), prev_bias);

     prev_state = pred_state;
     // prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));
     // // Reset the preintegration object.
     // imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
}


void iSAM2::update_gps(){

     correction_count++;
     Eigen::Matrix<float,7,1> gps = Eigen::Matrix<float,7,1>::Zero();


     // Adding IMU factor and GPS factor and optimizing.

     // Now optimize and compare results.


     // LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
     // Values result = optimizer.optimize();
     //
     // // Overwrite the beginning of the preintegration for the next step.
     // prev_state = NavState(result.at<Pose3>(X(correction_count)),
     //                      result.at<Vector3>(V(correction_count)));
     // prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));
     //
     // // Reset the preintegration object.
     // imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

     // Print out the position and orientation error for comparison.
     Vector3 gtsam_position = prev_state.pose().translation();
     // Vector3 position_error = gtsam_position - gps.head<3>();
     // current_position_error = position_error.norm();

     Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
     // Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
     // Quaternion quat_error = gtsam_quat * gps_quat.inverse();
     // quat_error.normalize();
     // Vector3 euler_angle_error(quat_error.x()*2,
     //                           quat_error.y()*2,
     //                           quat_error.z()*2);
     // current_orientation_error = euler_angle_error.norm();
     //
     // fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
     //        output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
     //        gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
     //        gps(0), gps(1), gps(2),
     //        gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());
     //
     // output_time += 1.0;


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

     // Reset the preintegration object.
     imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);

     // Print timings
     // tictoc_print_();

     // cout << "****************************************************" << endl;
	// currentEstimate.print("Current estimate: ");
	// Pose2 curEstimate = currentEstimate.at<Pose2>(Symbol('X', num_odom_updates));
     prev_state = NavState(currentEstimate.at<Pose3>(X(num_imu_updates)),
                          currentEstimate.at<Vector3>(V(num_imu_updates)));
     prev_bias = currentEstimate.at<imuBias::ConstantBias>(B(num_imu_updates));
     Vector3 gtsam_position = prev_state.pose().translation();
     prev_state.print("Latest Estimate: ");

	// Clear the factor graph and values for the next iteration
	graph.resize(0);
	initialEstimate.clear();
}

void iSAM2::predict(){}
void iSAM2::model(){}

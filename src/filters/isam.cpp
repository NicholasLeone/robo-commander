#include "isam.h"

using namespace std;
using namespace gtsam;

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
     pose_noise_model = NM::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
     velocity_noise_model = NM::Isotropic::Sigma(3,0.1); // m/s
     bias_noise_model = NM::Isotropic::Sigma(6,1e-3);

     /**------------- IMU Section --------------*/
     // Assemble initial quaternion through gtsam constructor ::quaternion(w,x,y,z);
     Rot3 prior_rotation = Rot3::Quaternion(1, 0, 0, 0);
     Point3 prior_point(0, 0, 0);
     Pose3 prior_pose(prior_rotation, prior_point);
     Vector3 prior_velocity(0, 0, 0);
     initPose = prevPose = Pose2(initX[0], initX[1], initX[2]);

     // Store Initial poses into the 'Values' containers for later analysis
     // initialEstimate.insert(Symbol('X', 0), initPose);
     initialEstimate.insert(X(correction_count), prior_pose);
     initialEstimate.insert(V(correction_count), prior_velocity);
     initialEstimate.insert(B(correction_count), prior_imu_bias);

     // Add prior poses to the factor graph
     // graph.push_back(PriorFactor<Pose2>(Symbol('X', 0), initPose, priorNoise));
     graph.add(PriorFactor<Pose3>(X(correction_count), prior_pose, pose_noise_model));
     graph.add(PriorFactor<Vector3>(V(correction_count), prior_velocity,velocity_noise_model));
     graph.add(PriorFactor<imuBias::ConstantBias>(B(correction_count), prior_imu_bias,bias_noise_model));

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
     prop_state = prev_state;
     prev_bias = prior_imu_bias;

     // Keep track of the total error over the entire run for a simple performance metric.
     current_position_error = current_orientation_error = 0.0;

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

void iSAM2::update_imu(vector<float> data){

     float ax, ay, az, gx, gy, gz, mx, my, mz, qx, qy, qz, qw, dt;

	// // Update Global counter
  	// num_imu_updates++;
     //
	// // Add IMU factor to the factor graph
  	// // graph.push_back(RangeFactor<Pose2, Pose2>(Symbol('A', old_odom_counter), Symbol('B', old_odom_counter_2), range_in, rangeNoise));
     //
	// // Perform one iSAM2 update step and get the current estimated poses
	// isam.update(graph, initialEstimate);
	// currentEstimate = isam.calculateEstimate();
     //
	// // Print out estimate for debug
	// // currentEstimate.print("Current Estimate");
     //
	// // Retrieve most recent estimate of each agent
	// Pose2 curEstimate = currentEstimate.at<Pose2>(Symbol('X', num_updates));
     //
	// // Reset factor graph and initial estimate values for next update step
	// graph.resize(0);
	// initialEstimate.clear();

     /**==============================================*/
     // IMU measurement
     // Eigen::Matrix<float,6,1> imu = Eigen::Matrix<float,6,1>::Zero();
     // for(int i=1; i<7; ++i){
     //      imu(i-1) = data.at(i);
     // }
     //
     // // Adding the IMU preintegration.
     // imu_preintegrated_->integrateMeasurement(imu.head<3>(), imu.tail<3>(), dt);

     // }else if(type == 1){ // GPS measurement
     //      Eigen::Matrix<float,7,1> gps = Eigen::Matrix<float,7,1>::Zero();
     //      // for (int i=0; i<6; ++i) {
     //      //   gps(i) = atof(value.c_str());
     //      // }
     //      // gps(6) = atof(value.c_str());
     //
     //      correction_count++;
     //
     //      // Adding IMU factor and GPS factor and optimizing.
     // #ifdef USE_COMBINED
     //      PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<PreintegratedCombinedMeasurements*>(imu_preintegrated_);
     //      CombinedImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
     //                                   X(correction_count  ), V(correction_count  ),
     //                                   B(correction_count-1), B(correction_count  ),
     //                                   *preint_imu_combined);
     //      graph.add(imu_factor);
     // #else
     //      PreintegratedImuMeasurements *preint_imu = dynamic_cast<PreintegratedImuMeasurements*>(imu_preintegrated_);
     //      ImuFactor imu_factor(X(correction_count-1), V(correction_count-1),
     //                           X(correction_count  ), V(correction_count  ),
     //                           B(correction_count-1),
     //                           *preint_imu);
     //      graph.add(imu_factor);
     //      imuBias::ConstantBias zero_bias(Vector3(0, 0, 0), Vector3(0, 0, 0));
     //      graph.add(BetweenFactor<imuBias::ConstantBias>(B(correction_count-1),
     //                                                      B(correction_count  ),
     //                                                      zero_bias, bias_noise_model));
     // #endif
     //
     //      NM::Diagonal::shared_ptr correction_noise = NM::Isotropic::Sigma(3,1.0);
     //      // GPSFactor gps_factor(X(correction_count),
     //      //                      Point3(gps(0),  // N,
     //      //                             gps(1),  // E,
     //      //                             gps(2)), // D,
     //      //                      correction_noise);
     //      // graph.add(gps_factor);
     //
     //      // Now optimize and compare results.
     //      prop_state = imu_preintegrated_->predict(prev_state, prev_bias);
     //      initialEstimate.insert(X(correction_count), prop_state.pose());
     //      initialEstimate.insert(V(correction_count), prop_state.v());
     //      initialEstimate.insert(B(correction_count), prev_bias);
     //
     //      LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
     //      Values result = optimizer.optimize();
     //
     //      // Overwrite the beginning of the preintegration for the next step.
     //      prev_state = NavState(result.at<Pose3>(X(correction_count)),
     //                           result.at<Vector3>(V(correction_count)));
     //      prev_bias = result.at<imuBias::ConstantBias>(B(correction_count));
     //
     //      // Reset the preintegration object.
     //      imu_preintegrated_->resetIntegrationAndSetBias(prev_bias);
     //
     //      // Print out the position and orientation error for comparison.
     //      Vector3 gtsam_position = prev_state.pose().translation();
     //      Vector3 position_error = gtsam_position - gps.head<3>();
     //      current_position_error = position_error.norm();
     //
     //      Quaternion gtsam_quat = prev_state.pose().rotation().toQuaternion();
     //      Quaternion gps_quat(gps(6), gps(3), gps(4), gps(5));
     //      Quaternion quat_error = gtsam_quat * gps_quat.inverse();
     //      quat_error.normalize();
     //      Vector3 euler_angle_error(quat_error.x()*2,
     //                                quat_error.y()*2,
     //                                quat_error.z()*2);
     //      current_orientation_error = euler_angle_error.norm();
     //
     //      fprintf(fp_out, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
     //             output_time, gtsam_position(0), gtsam_position(1), gtsam_position(2),
     //             gtsam_quat.x(), gtsam_quat.y(), gtsam_quat.z(), gtsam_quat.w(),
     //             gps(0), gps(1), gps(2),
     //             gps_quat.x(), gps_quat.y(), gps_quat.z(), gps_quat.w());
     //
     //      output_time += 1.0;
     //
     // }


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

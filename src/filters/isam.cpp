#include "isam.h"

using namespace std;
using namespace gtsam;
namespace NM = gtsam::noiseModel;

iSAM2::iSAM2(){

     // Initialize variables
     oldX[0] = initX[0]; oldX[1] = initX[1]; oldX[2] = initX[2];
     flag_odom_initialized = false;
     fresh_odom = false;

     // Debugging
     printf("[INITIALS]:\r\n  X, Y, Z: %.4f, %.4f, %.4f\r\n",initX[0], initX[1], initX[2]);
     printf("[PREVIOUS]:\r\n  X, Y, Z: %.4f, %.4f, %.4f\r\n",oldX[0], oldX[1], oldX[2]);

     /** Define our iSAM2 parameters */
     parameters.relinearizeThreshold = 0.01;
     parameters.relinearizeSkip = 1;
 	ISAM2 isam(parameters);

	// Initialize noise models to be used by GTSAM libraries
	Vector priorSigmas = Vector3(priorSig,priorSig,priorSig);

     priorNoise = NM::Diagonal::Sigmas(priorSigmas);  //prior
	imuNoise = NM::Isotropic::Sigma(1, sigmaR);      // non-robust

     // Add prior poses to the factor graph
	initPose = prevPose = Pose2(initX[0], initX[1], initX[2]);
	graph.push_back(PriorFactor<Pose2>(Symbol('A', 0), initPose, priorNoise));

     // Store Initial poses into the 'Values' containers for later analysis
	initialEstimate.insert(Symbol('A', 0), initPose);
	// Print out stored variables for debugging
	initialEstimate.print("Initial Pose: ");

     /** Initialize Misc Variables */


}

iSAM2::~iSAM2(){}

void iSAM2::update_odometry(float deltas[3]){
     num_odom++;

     int odom_counter;
	float temp_x, temp_y, qx, qy, qz, qw, yaw;
	float x, y, dist_traveled;
	float covX, covY, covYaw;
	float dx, dy, dyaw;
	Pose2 newPose;

     odometry = Pose2(deltas[0], deltas[1], deltas[2]);

     // Update Global counter
	numOdomUno = numOdomUno + 1;

	// Load up our incoming data locally
	// odom_counter_1 = msg->header.seq;
     //
	// temp_x = msg->pose.pose.position.x;
	// temp_y = msg->pose.pose.position.y;
	// qx = msg->pose.pose.orientation.x;
	// qy = msg->pose.pose.orientation.y;
	// qz = msg->pose.pose.orientation.z;
	// qw = msg->pose.pose.orientation.w;
     //
	// covX = msg->pose.covariance[0];
	// covY = msg->pose.covariance[7];
	// covYaw = msg->pose.covariance[35];

	// Setup the noise model
	Vector odoCov = Vector3(covX,covY,covYaw);
	odomNoise = NM::Diagonal::Variances(odoCov);

	// Calculate yaw from quaternions
	yaw = atan2(2*qw*qz+2*qx*qy,1-2*qy*qy-2*qz*qz);

	rawOdom << temp_x << endr << temp_y << endr;

	// Extract data from matrix into usable form
	rawOdom = rawOdom.t();
	odom_x = rawOdom.col(0);
	odom_y = rawOdom.col(1);

	// Put Rotated Odom data into global frame
    	x = x1_init + odom_x(0);
    	y = y1_init + odom_y(0);
    	yaw = yaw1_init + yaw;

	// Calculate change in pose since our last recorded odometry pose
	dx = x - old_x1;
	dy = y - old_y1;
	dyaw = yaw - old_yaw1;
	dist_traveled = sqrt(dx*dx + dy*dy);

	// BetweenFactor was found to work when only using the change in pose relative to the body frame
	odometryUno = Pose2(dist_traveled, 0, dyaw);

	// Store important variables for later analysis
	// TODO: re-organization of what we want to use for later analysis
	newPose = Pose2(x, y, yaw);
	odomValues.insert(numOdomUno, odometryUno);

	// Ensure we load the BetweenFactor in the factor graph with proper linking of keys
    	if(flag_odom_initialized) {
        	graph.push_back(BetweenFactor<Pose2>(Symbol('A',old_odom_counter), Symbol('A',odom_counter), odometry, odomNoise));
	} else {
        	graph.push_back(BetweenFactor<Pose2>(Symbol('A',0), Symbol('A',odom_counter), odometry, odomNoise));
	}

	/** NOTE:
		Using simulated control inputs from the changes in the estimated pose
	recieved from the dead-reckoned path easily obtained from Turtlebot,
	predict our new pose estimate from our last known pose
	*/
    	Pose2 predPose = prev.compose(odometry);

	// Add this predicted pose to our initial estimate
	initialEstimate.insert(Symbol('A', odom_counter), predPose);

	/** Print out any debug values */
	// cout << "[AGENT 1] Covariance: " << covX << "	" << covY << " 	" << covYaw << endl;

	// Update Global variables
	old_x = x;
	old_y = y;
	old_yaw = yaw;
	prevPose = predPose;

	old_odom_counter = odom_counter;
	flag_odom_initialized = true;

}

void iSAM2::update_odometry(float deltas[2]){
     num_odom++;

     odometry = Pose2(deltas[0], 0, deltas[1]);

}

void iSAM2::update_imu(){

	float xhat, yhat;
	float error, dx, dy;

	// Update Global counter
  	num_imu++;

	// Add IMU factor to the factor graph
  	// graph.push_back(RangeFactor<Pose2, Pose2>(Symbol('A', old_odom_counter), Symbol('B', old_odom_counter_2), range_in, rangeNoise));

	// Perform one iSAM2 update step and get the current estimated poses
	isam.update(graph, initialEstimate);
	currentEstimate = isam.calculateEstimate();

	// Print out estimate for debug
	// currentEstimate.print("Current Estimate");

	// Retrieve most recent estimate of each agent
	Pose2 pose1 = currentEstimate.at<Pose2>(Symbol('P', old_odom_counter));
	xhat = pose.x();
	yhat = pose.y();


	// Calculate the estimate error compared to truth
	dx = true_x - xhat;
	dy = true_y - yhat;

	error = sqrt(dx*dx + dy*dy);

	SSE = SSE + error*error;

	// Reset factor graph and initial estimate values for next update step
	graph.resize(0);
	initialEstimate.clear();


}


void iSAM2::update(){
     // gttic_(update);
	// // Update iSAM with the new factors
	// isam.update(graph, initialEstimate);
	// // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
	// // If accuracy is desired at the expense of time, update(*) can be called additional times
	// // to perform multiple optimizer iterations every step.
	// isam.update();
	// gttoc_(update);
	// gttic_(calculateEstimate);
	// currentEstimate = isam.calculateEstimate();
	// gttoc_(calculateEstimate);
	// cout << "****************************************************" << endl;
	// currentEstimate.print("Current estimate: ");
	//
	// // Print timings
	// tictoc_print_();
	//
	// // Clear the factor graph and values for the next iteration
	// graph.resize(0);
	// initialEstimate.clear();
}

void iSAM2::predict(){}
void iSAM2::model(){}

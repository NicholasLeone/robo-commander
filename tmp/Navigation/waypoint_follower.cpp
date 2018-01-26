#include "waypoint_follower.h"
#include <math.h>
#include <armadillo>
// TODO: somehow incorporate this via the 'Agent' class later
#define MAX_ANGULAR_VELOCITY 3.0
#define MAX_LINEAR_VELOCITY 1.0
#define LOOKAHEAD_DIST 1

/** ============================
*		TODO: Put this and other miscellaneous functions in a seperate folder
* ============================== */

float saturate(fmat omega) {
	float sat_vel;

	if(as_scalar(abs(omega)) > MAX_ANGULAR_VELOCITY)
		sat_vel = as_scalar(sign(omega) * MAX_ANGULAR_VELOCITY);
	else
		sat_vel = as_scalar(omega);

	return sat_vel;
}

float calculateDistance(fmat targetCoordinates, fmat currentCoordinates){
	float curX, curY, TargetX, TargetY, dx, dy, distance;

	curX = currentCoordinates(0);
	curY = currentCoordinates(1);
	TargetX = targetCoordinates(0);
	TargetY = targetCoordinates(1);

	dx = TargetX - curX;
	dy = TargetY - curY;
	distance = sqrt(dx*dx + dy*dy);

	return distance;

}
/** ============================
*		END TODO
* ============================== */

WaypointFollower::WaypointFollower(fmat wayList, float goalRadius){
	waypoints = wayList;
	epsilon = goalRadius;

	// Variable Initialization
	curIndex = 0;
	goalReached = false;
	destDist = 100000; 		// Initially wrong for looping conditions

	// Extrapolated data from inputs
	goal = waypoints.tail_rows(1);
	numPts = waypoints.n_rows;

	// Debug Output
	cout << "[FOLLOWER INITIALIZED]: " << "# of Waypoints: " << numPts << "	";
	cout << "Epsilon: " << epsilon << endl;

	goal.print("Agent Goal: ");
}

fmat WaypointFollower::updateControls(fmat target, fmat curPose) {

     float x, y, Gx, Gy, yaw;
	fmat w, controls, v;

     Gx = target(0); Gy = target(1);
     x = curPose(0); y = curPose(1); yaw = curPose(2);

     // Calculate Euclidean distance from current position to goal location
     float dist = calculateDistance(target, curPose);

     // Angle between agent heading and the line connecting agent and target
	float slope = atan2((Gy - y), (Gx - x));
	float alpha = slope - yaw;

	// cout << "WF: SLOPE, ALPHA: " << slope << "	, " << alpha << endl;

	// Angular velocity command for differential drive agent
	// 		is equal to the desired curvature to be followed by the robot.
	w = (2 * sin(alpha)) / LOOKAHEAD_DIST;

	// Pick constant rotation if robot is facing opposite direction of the path
	if(abs(abs(alpha) - datum::pi) < 1e-12)
		w = sign(w) * 1;

	float w_val = saturate(w);
	float v_val = dist * 100 * cos(alpha);
	//float v_val = cos(alpha) / LOOKAHEAD_DIST;
	v << v_val << endr;

	if(v_val > MAX_LINEAR_VELOCITY)
		v_val = as_scalar(sign(v)) * MAX_LINEAR_VELOCITY;

	// controls << v_val << w_val << endr;
	controls << MAX_LINEAR_VELOCITY << w_val << endr;
	//controls.print("Controls: ");
	//cout << "Controls Size: " << size(controls) << endl;

	return controls;
}


fmat WaypointFollower::updateTarget(fmat poseData) {

	fmat XY = poseData.head_cols(2);
	fmat tmpTarget = waypoints.row(curIndex);
	// cout << "Size XY; target: " << size(XY) << "	;	" << size(tmpTarget) << endl;

	float targetDist = calculateDistance(tmpTarget,XY);

	// Update distance to goal TODO: make available outside this function
	destDist = calculateDistance(goal, XY);

	cout << "Index: " << curIndex << "	Distance: " << targetDist << endl;

	if(targetDist <= epsilon){
		if(curIndex < numPts-1){
			curIndex++;
			goalReached = false;
		}

		if(curIndex >= numPts-1){
			curIndex = numPts-1;
			goalReached = true;
		}
	}

	return tmpTarget = waypoints.row(curIndex);

}

fmat WaypointFollower::updateHistory(fmat prevHistory,fmat pose,fmat controls){
	fmat dataMux, newHistory;

	dataMux = join_rows(pose, controls);
	newHistory = join_cols(prevHistory, dataMux);

	return newHistory;
}

WaypointFollower::~WaypointFollower(){

}

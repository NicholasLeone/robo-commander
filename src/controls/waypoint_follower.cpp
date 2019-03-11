#include <iostream>

#include "waypoint_follower/matplotlibcpp.h"
#include "waypoint_follower/waypoint_follower.h"

namespace plt = matplotlibcpp;
using namespace std;
using namespace arma;

WaypointFollower::WaypointFollower(){
	this->_goal << 0 << 0 << endr;
	this->_cur_target << 0 << 0 << endr;
}
WaypointFollower::WaypointFollower(fmat ref_path){}
WaypointFollower::~WaypointFollower(){}

/** -----------------------
*	Private Functions
* -------------------------- */
fmat WaypointFollower::_get_nearest_waypoint(fmat pose2d, bool verbose){
	uword nearIdx;

	fmat path = this->_reference_path;
	fmat x_pts = pow((path.col(0) - pose2d(0,0)),2);
	fmat y_pts = pow((path.col(1) - pose2d(0,1)),2);
	fmat dist = sqrt(x_pts + y_pts);
	float min_val = dist.min(nearIdx);
	fmat coords = path.row(nearIdx);

	if(verbose){
		cout << "Size of Distances: " << dist.n_rows << ", " << dist.n_cols << endl;
		cout << "min val: " << min_val << "	@: " << nearIdx << endl;
		coords.print("Closest Waypoint XY: ");
	}
	return coords;
}

uword WaypointFollower::_get_nearest_waypoint_index(fmat pose2d, bool verbose){
	uword nearIdx;

	fmat path = this->_reference_path;
	fmat x_pts = pow((path.col(0) - pose2d(0,0)),2);
	fmat y_pts = pow((path.col(1) - pose2d(0,1)),2);
	fmat dist = sqrt(x_pts + y_pts);
	float min_val = dist.min(nearIdx);

	if(verbose){
		cout << "Size of Distances: " << dist.n_rows << ", " << dist.n_cols << endl;
		cout << "min val: " << min_val << "	@: " << nearIdx << endl;
	}
	return nearIdx;
}

int WaypointFollower::_update_target_index(fmat pose2d){
	int status;
	uword search_index, last_index, nearest_index;
	ucolvec chosen_index;

	// If there aren't any search points create one, otherwise use last search point
	if(this->_target_idx_history.is_empty()){
		chosen_index = this->_get_nearest_waypoint_index(pose2d);
		this->_target_idx_history = join_cols(this->_target_idx_history, chosen_index);
		status = 0;
	} else{
		// Grab the last most recorded lookahead index
		last_index = as_scalar(this->_target_idx_history.tail(1));
		// Search for acceptable lookahead indices
		int search_dist = 2 * this->_lookahead_index_distance;
		if(last_index + search_dist < this->_reference_path.n_rows)
			search_index = last_index + search_dist;
		else search_index = this->_reference_path.n_rows - 1;

		// Grab all the indices from the reference path that're within the search range
		fmat dxs = this->_reference_path(span(last_index,search_index),0) - pose2d(0,0);
		fmat dys = this->_reference_path(span(last_index,search_index),1) - pose2d(0,1);

		// Find the respective Euclidean distances for each point
		fmat dists = sqrt(square(dxs) + square(dys));
		float min_val = dists.min(nearest_index);
		chosen_index = last_index + nearest_index;
		this->_target_idx_history = join_cols(this->_target_idx_history, chosen_index);
		status = 1;
	}
	return status;
}


void WaypointFollower::_update_target_waypoint(fmat pose2d, bool verbose){

	// Do the following if we are following a pre-defined path
	if(this->follow_mode == 0){
		this->_lookahead_index_distance = round(this->_lookahead_dist / this->_min_increment_dist);
		if(verbose) cout << "Lookahead Index Distance: " << this->_lookahead_index_distance << endl;

		int status = this->_update_target_index(pose2d);
		if(status == 0){
			int lookahead_point_index = as_scalar(this->_target_idx_history.tail(1));
			this->_cur_target = this->_reference_path.row(lookahead_point_index);
			if(verbose) cout << "No Previous Targets.\r\n 	 New Target Found at index: " << lookahead_point_index << endl;
		}else{
			int lookahead_point_index = as_scalar(this->_target_idx_history.tail(1)) + this->_lookahead_index_distance;

			// If the proposed target index is within bounds then use the target at that index
			if(lookahead_point_index < this->_reference_path.n_rows){
				this->_cur_target = this->_reference_path.row(lookahead_point_index);
				if(verbose) cout << "Updated Target Index: " << lookahead_point_index << endl;
			} else{ // Otherwise use the last waypoint from the reference path
				this->_cur_target = this->_reference_path.tail_rows(1);
				if(verbose) cout << "Using Last Reference Index: " << this->_reference_path.n_rows << endl;
			}
		}
	}else{             // Otherwise follow a single waypoint updated externally
		if(verbose) cout << "Updated Target Waypoint: " << this->_cur_target << endl;
	}
	if(verbose) cout << "Updated Target Waypoint: " << this->_cur_target << endl;

	// Record updated target information (index and waypoint at index)
	this->_target_history = join_cols(this->_target_history, this->_cur_target);
}

fmat WaypointFollower::_saturate_controls(fmat raw_controls){
	fmat safe_controls;
	float safe_w, safe_v;

	float v = as_scalar(raw_controls(0));
	float w = as_scalar(raw_controls(1));

	// Ensure Linear velocity does not exceed maximum defined
	if(v > this->_maxv) safe_v = copysign(this->_maxv , v);
	else safe_v = v;

	// Ensure Angular velocity does not exceed maximum defined
	if(w > this->_maxw) safe_w = copysign(this->_maxw , w);
	else safe_w = w;

	// Re-package filtered controls
	safe_controls << safe_v << safe_w << endr;
	return safe_controls;
}

/** -----------------------
*	Public Functions
* -------------------------- */
void WaypointFollower::load_path(string file_path, bool switch_xy, bool verbose, bool plot){
	fmat traj;
	traj.load(file_path);

	this->_reference_path = traj;

	if(switch_xy){
		// this->_reference_path.swap_cols(0,1);
	}

	fmat dpath = diff(traj);
	fmat dists = sqrt(pow(dpath.col(0),2) + pow(dpath.col(1),2));
	fmat sort_dists = dists;
	sort_dists.elem( find(sort_dists < 0.0001) ).zeros();
	sort_dists = nonzeros(sort_dists);
	this->_min_increment_dist = sort_dists.min();
	this->_goal = traj.tail_rows(1);

	if(verbose){
		cout << "	WaypointFollower --- Loaded Path Size: " << traj.n_rows << ", " << traj.n_cols << endl;
		this->_goal.print("	WaypointFollower --- Loaded Path --- Goal: ");
		sort_dists.print("Loaded Path --- Nonzero Path Inter-Path Distances: ");
		cout << "	WaypointFollower --- Loaded Path --- min_increment_distance: " << this->_min_increment_dist << endl;
	}
	if(plot) this->plot_paths();
}

void WaypointFollower::update(vector<float> cur_pose2d){
	fmat pose;
	pose << cur_pose2d.at(0) << cur_pose2d.at(1) << cur_pose2d.at(2) << endr;
	this->_update_target_waypoint(pose);

	// Store Current Pose in pose history for visualizing traversed trajectory
	this->_pose_history = join_cols(this->_pose_history, pose);
}


vector<float> WaypointFollower::get_commands(vector<float> cur_pose2d, bool verbose){
	fmat raw_controls, controls, pose;
	fmat target = this->_cur_target;
	pose << cur_pose2d.at(0) << cur_pose2d.at(1) << cur_pose2d.at(2) << endr;

	float dx = target(0) - pose(0);
	float dy = target(1) - pose(1);

	// Angle between robot heading and the line connecting robot and the current target waypoint
	float slope = atan2(dy, dx);
	float alpha = slope - pose(2);

	float vraw = this->_maxv;
	// Angular velocity command for a differential drive robot is equal to the desired curvature to be followed by the robot.
	float wraw = ( 2 * sin(alpha) ) / this->_lookahead_dist;

	// Pick a constant rotation when robot is facing in the opposite direction of the path
	if(abs(abs(alpha) - datum::pi) < 1e-12) wraw = copysign(1.0, wraw);

	raw_controls << vraw << wraw << endr;
	controls = this->_saturate_controls(raw_controls);

	if(verbose){
		// cout << "LookAheadPoint: " << lookAheadPoint << "			Slope to Path: " << slope << "				Alpha: " << alpha << endl;
		// cout << "Controls Size: " << size(controls) << endl;
		controls.print("	WaypointFollower --- Controls: ");
	}

	vector<float> vec_controls = conv_to<vecf_t>::from(controls.row(0));
	return vec_controls;
}



/** -----------------------
*	Setter Functions
* -------------------------- */
void WaypointFollower::set_follow_mode(int mode){this->follow_mode = mode;}
void WaypointFollower::set_lookahead_distance(float distance){this->_lookahead_dist = distance;}
void WaypointFollower::set_goal_radius_threshold(float radius){this->_goal_radius_threshold = radius;}
void WaypointFollower::set_target_radius_threshold(float radius){this->_target_radius_threshold = radius;}
void WaypointFollower::set_reference_path(fmat ref_path){this->_reference_path = ref_path;}
void WaypointFollower::set_target(vector<float> xy_target){
	fmat target;
	target << xy_target.at(0) << xy_target.at(1) << endr;
	this->_cur_target = target;
}
void WaypointFollower::set_goal(vector<float> goal_pose2d){
	fmat goal;
	goal << goal_pose2d.at(0) << goal_pose2d.at(1) << endr;
	this->_goal = goal;
}
void WaypointFollower::set_max_commands(vector<float> limits, bool verbose){
	this->_maxv = limits.at(0);
	this->_maxw = limits.at(1);
	if(verbose) cout << "	WaypointFollower --- Set Max Commands: " << this->_maxv << ", " << this->_maxw << endl;
}

/** -----------------------
*	Getter Functions
* -------------------------- */
float WaypointFollower::get_distance_to_goal(vector<float> cur_pose2d, bool verbose){
	fmat pose;
	pose << cur_pose2d.at(0) << cur_pose2d.at(1) << cur_pose2d.at(2) << endr;
	fmat goal = this->_goal.head_cols(2);
	fmat loc = pose.head_cols(2);
	this->_distance2goal = norm(loc - goal);

	if(verbose){
		pose.print("	WaypointFollower --- get_distance_to_goal --- Current Pose: ");
		loc.print("	WaypointFollower --- get_distance_to_goal --- Current Location: ");
		goal.print("	WaypointFollower --- get_distance_to_goal --- Current Goal: ");
		cout << "	WaypointFollower --- get_distance_to_goal --- distanceToGoal: " << this->_distance2goal << endl;
	}
	return this->_distance2goal;
}

float WaypointFollower::get_goal_radius_threshold(){ return this->_goal_radius_threshold;}

vector<float> WaypointFollower::get_current_target(){
	vector<float> target = conv_to<vecf_t>::from(this->_cur_target);
	return target;
}

vector<float> WaypointFollower::get_goal(){
	vector<float> goal = conv_to<vecf_t>::from(this->_goal);
	return goal;
}

mat_t WaypointFollower::get_pose_history(bool verbose){
	fmat tmp = this->_pose_history;
	int rs = tmp.n_rows, cs = tmp.n_cols;
	if(verbose) cout << "Size of PoseHistory: " << rs << ", " << cs << endl;

	mat_t out(rs, vector<float>(cs));
	if(verbose) cout << "Created Matrix:\r\n";
	for(int row = 0; row<rs;row++){
		if(verbose) cout << "		";
		for(int col = 0;col<cs; col++){
			out[row][col] = tmp(row,col);
			if(verbose) cout << out[row][col] << "		";
		}
		if(verbose) cout << "\r\n";
	}
	return out;
}

mat_t WaypointFollower::get_reference_path(bool verbose){
	fmat tmp = this->_reference_path;
	int rs = tmp.n_rows, cs = tmp.n_cols;
	if(verbose) cout << "Size of Reference Path: " << rs << ", " << cs << endl;

	mat_t out(rs, vector<float>(cs));
	if(verbose) cout << "Created Matrix:\r\n";
	for(int row = 0; row<rs;row++){
		if(verbose) cout << "		";
		for(int col = 0;col<cs; col++){
			out[row][col] = tmp(row,col);
			if(verbose) cout << out[row][col] << "		";
		}
		if(verbose) cout << "\r\n";
	}
	return out;
}

/** -----------------------
*	Misc Functions
* -------------------------- */
bool WaypointFollower::isGoalReached(vector<float> cur_pose2d){
	float d2g = this->get_distance_to_goal(cur_pose2d);
	if(d2g <= this->_goal_radius_threshold) return true;
	else return false;
}

void WaypointFollower::plot_paths(){
	fmat traj = this->_reference_path;
	vector<float> xs, ys;
	xs = conv_to<vecf_t>::from(traj.col(0));
	ys = conv_to<vecf_t>::from(traj.col(1));
	plt::plot(xs,ys,"b");
	plt::show();
}

void WaypointFollower::plot_pose_w_paths(fmat pose_2d){
	fmat traj = this->_reference_path;
	fmat goal = this->_goal;
	vecf_t xs, ys, px, py, gx, gy;
	xs = conv_to<vecf_t>::from(traj.col(0));
	ys = conv_to<vecf_t>::from(traj.col(1));
	px = conv_to<vecf_t>::from(pose_2d.col(0));
	py = conv_to<vecf_t>::from(pose_2d.col(1));
	gx = conv_to<vecf_t>::from(goal.col(0));
	gy = conv_to<vecf_t>::from(goal.col(1));

	plt::plot(xs,ys,"b", px, py, "r+", gx, gy, "gs");
	plt::show();
}

void WaypointFollower::plot_pose_w_paths(vector<float> pose_2d){
	fmat traj = this->_reference_path;
	fmat goal = this->_goal;
	vecf_t xs, ys, px, py, p, gx, gy;
	p = pose_2d;
	xs = conv_to<vecf_t>::from(traj.col(0));
	ys = conv_to<vecf_t>::from(traj.col(1));

	gx = conv_to<vecf_t>::from(goal.col(0));
	gy = conv_to<vecf_t>::from(goal.col(1));

	px.push_back(pose_2d.at(0));
	py.push_back(pose_2d.at(1));

	plt::plot(xs,ys,"b", px, py, "r+", gx, gy, "gs");
	plt::show();
}

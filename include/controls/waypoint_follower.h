#ifndef WAYPOINT_FOLLOWER_H_
#define WAYPOINT_FOLLOWER_H_

#include <armadillo>
#include <chrono>
#include <vector>
#include <cmath>

using namespace std;
using namespace arma;

struct CoordinatePair{
	float x;
	float y;
};

typedef chrono::high_resolution_clock fineclock_t;
typedef vector<float> vecf_t;
typedef vector< vector<float> > mat_t;

class WaypointFollower{
private:
	// Tuning Parameters
	float _lookahead_dist = 0.5;
	float _min_increment_dist;
	float _goal_radius_threshold;
	float _target_radius_threshold;
	float _maxv = 0.5;
	float _maxw = 3.14;

	int follow_mode = 0;
	int _lookahead_index_distance;

	fineclock_t::time_point _prev_time;
	fineclock_t::time_point _cur_time;
	// std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();]
	//  = high_resolution_clock::now();

	// Internal parameters
	float _distance2goal;
	float _dt;
	fmat _reference_path;
	fmat _cur_target;
	fmat _target_history;
	fmat _pose_history;
	ucolvec _target_idx_history;
	fmat _goal;

	/** --------------
	* 	Functions
	* --------------- */
	fmat _saturate_controls(fmat raw_controls);
	fmat _get_nearest_waypoint(fmat pose2d, bool verbose = false);
	uword _get_nearest_waypoint_index(fmat pose2d, bool verbose = false);
	int _update_target_index(fmat pose2d);
	void _update_target_waypoint(fmat pose2d, bool verbose = true);

public:
	WaypointFollower();
	WaypointFollower(fmat ref_path);
	~WaypointFollower();

	// Controller update function
	void load_path(string file_path, bool switch_xy = false, bool verbose = false, bool plot = false);
	void update(vector<float> cur_pose2d);
	vector<float> get_commands(vector<float> cur_pose2d, bool verbose = false);

	// Setters
	void set_follow_mode(int mode);
	void set_lookahead_distance(float distance);
	void set_goal_radius_threshold(float radius);
	void set_target_radius_threshold(float radius);
	void set_reference_path(fmat ref_path);
	void set_target(vector<float> xy_target);
	void set_goal(vector<float> goal_pose2d);
	void set_max_commands(vector<float> limits, bool verbose = false);

	// Getters
	float get_distance_to_goal(vector<float> cur_pose2d, bool verbose = false);
	float get_goal_radius_threshold();
	vector<float> get_current_target();
	vector<float> get_goal();
	mat_t get_pose_history(bool verbose = false);
	mat_t get_reference_path(bool verbose = false);

	// Flaggers
	bool isGoalReached(vector<float> cur_pose2d);

	// Debugging and Visualization
	void plot_paths();
	void plot_pose_w_paths(fmat pose_2d);
	void plot_pose_w_paths(vector<float> pose_2d);

}; // WaypointFollower

#endif // WAYPOINT_FOLLOWER_H_

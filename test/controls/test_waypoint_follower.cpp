#include <iostream>
#include "controls/waypoint_follower.h"
#include <vector>

using namespace std;

int main(){
	vector<float> pose0 = {-1, -1, 0};
	// pose0 << -1 << -1 << endr;

	WaypointFollower wf;

	wf.load_path("/home/hunter/Documents/key_waypoints.csv");
	wf.update_target_waypoint(pose0);
	vector<float> cmds = wf.get_commands(pose0, true);

	float d2g = wf.get_distance_to_goal(pose0, true);

	wf.plot_pose_w_paths(pose0);

	return 1;
}

#ifndef WAYPOINT_FOLLOWER_H_
#define WAYPOINT_FOLLOWER_H_

#include <armadillo>
 
using namespace std;
using namespace arma;

class WaypointFollower{

private:

     fmat waypoints;
     fmat goal;
     float epsilon;
     int curIndex;
     int numPts;

public:

     bool goalReached;
     float destDist;

     WaypointFollower(fmat wayList, float goalRadius);
     ~WaypointFollower();

     //TODO: replace pose with Agent class containing pose and velocity data
     fmat updateControls(fmat target, fmat pose);
     fmat updateTarget(fmat poseData);
     fmat updateHistory(fmat prevHistory, fmat pose, fmat controls);
};

#endif /** WAYPOINT_FOLLOWER_H_ */

#include <unistd.h>
#include <iostream>

#include "filters/isam.h"
#include "utils/utils.h"

using namespace std;
using namespace arma;

int main(){

	// VARIABLES
	string csv;
	int i = 0;
	float d[3];
	int n;
	// int t = 235; // Test row in csv

	string data_filename;
	if (argc < 2) {
		printf("Using Straight Line (~1m) .csv data file...\n");
		csv = "/home/hunter/Documents/Swanson/1m_straight_slow.csv";
		// csv = "/home/hunter/Documents/Swanson/360_turn_ccw_slow.csv";
	} else {
		csv = argv[1];
	}

	iSAM2 isam;

     fmat data = csv_to_matrix(csv);
	fmat times = data.col(0);
	fmat dDistances = data.col(10);
	fmat dYaws = data.col(11);

	for(int t = 0;t<data.n_rows;t++){
		d[0] = as_scalar(times.row(t));
		d[1] = as_scalar(dDistances.row(t));
		d[2] = as_scalar(dYaws.row(t));
		// printf("t, Δdistance (m), ΔYaw (rad): %.5f, %.8f,  %.8f\n\r", d[0], d[1], d[2]);
		isam.update_odometry(d[1], d[2]);
	}

	isam.update();


    	return 0;

}

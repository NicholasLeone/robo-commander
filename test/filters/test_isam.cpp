#include <unistd.h>
#include <iostream>

#include "filters/isam.h"
#include "utils/utils.h"

using namespace std;
using namespace arma;

int main(int argc, char *argv[]){

	// VARIABLES
	string csv;
	int i = 0;
	int n;
	float dt, axi, ayi, azi, gxi, gyi, gzi;
	float d[3];
	vector<float> initials;


	if (argc < 2) {
		printf("Using Straight Line (~1m) .csv data file...\n");
		csv = "/home/hunter/devel/robo-dev/data/fusion/1-0859m_straight_slow.csv";
		// csv = "/home/hunter/Documents/Swanson/1m_straight_slow.csv";
		// csv = "/home/hunter/Documents/Swanson/360_turn_ccw_slow.csv";
	} else {
		csv = argv[1];
	}

	iSAM2 isam;

	fmat times, qx, qy, qz, qw, ax, ay, az, gx, gy, gz, mx, my, mz, ex, ey, ez, dDistances, dYaws;
     fmat data = csv_to_matrix(csv);

	times = data.col(0);
	ax = data.col(1);
	ay = data.col(2);
	az = data.col(3);

	gx = data.col(4);
	gy = data.col(5);
	gz = data.col(6);

	mx = data.col(7);
	my = data.col(8);
	mz = data.col(9);

	qx = data.col(10);
	qy = data.col(11);
	qz = data.col(12);
	qw = data.col(13);

	ex = data.col(14);
	ey = data.col(15);
	ez = data.col(16);

	dDistances = data.col(17);
	dYaws = data.col(18);

	initials.push_back(0);
	initials.push_back(0);
	initials.push_back(0);
	initials.push_back(as_scalar(qx.row(0)));
	initials.push_back(as_scalar(qy.row(0)));
	initials.push_back(as_scalar(qz.row(0)));
	initials.push_back(as_scalar(qw.row(0)));

	isam.init(initials);

	for(int t = 1;t<data.n_rows;t++){
		vector<float> datai;

		dt = as_scalar(times.row(t) - times.row(t-1));
		axi = as_scalar(ax.row(t));
		ayi = as_scalar(ay.row(t));
		azi = as_scalar(az.row(t));
		gxi = as_scalar(gx.row(t));
		gyi = as_scalar(gy.row(t));
		gzi = as_scalar(gz.row(t));

		datai.push_back(dt);
		datai.push_back(axi);
		datai.push_back(ayi);
		datai.push_back(azi);
		datai.push_back(gxi);
		datai.push_back(gyi);
		datai.push_back(gzi);

		d[0] = as_scalar(times.row(t));
		d[1] = as_scalar(dDistances.row(t));
		d[2] = as_scalar(dYaws.row(t));

		// printf("t, Δdistance (m), ΔYaw (rad): %.5f, %.8f,  %.8f\n\r", d[0], d[1], d[2]);
		isam.update_odometry(d[1], d[2]);
		isam.update_imu(datai);
	}

	isam.update();

    	return 0;
}

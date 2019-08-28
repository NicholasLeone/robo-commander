#include <unistd.h>
#include <iostream>

#include "algorithms/isam.h"
#include "utilities/utils.h"

using namespace std;
using namespace arma;

int main(int argc, char *argv[]){

	// VARIABLES
	string csv;
	int i = 0;
	int n, flag_moved = 0;
	float dt, axi, ayi, azi, gxi, gyi, gzi;
	float d[3];
	vector<float> initials;
	string output_filename = "/home/hunter/devel/robo-dev/data/fusion/output/isam_outputs.csv";

	if (argc < 2) {
		printf("Using Straight Line (~1m) .csv data file...\n");
		csv = "/home/hunter/devel/robo-dev/data/fusion/1-0859m_straight_slow.csv";
		// csv = "/home/hunter/Documents/Swanson/1m_straight_slow.csv";
		// csv = "/home/hunter/Documents/Swanson/360_turn_ccw_slow.csv";
	} else {
		csv = argv[1];
	}

	iSAM2 isam;

	fmat times, qx, qy, qz, qw, ax, ay, az, gx, gy, gz, mx, my, mz, ex, ey, ez, dDistances, dYaws, dXs, dYs;
	fmat inputs,offsets;
	fmat rx, ry, rz;
     fmat data = csv_to_matrix(csv);

	times = data.col(0);
	ax = data.col(1); ay = data.col(2); az = data.col(3);
	gx = data.col(4); gy = data.col(5); gz = data.col(6);
	mx = data.col(7); my = data.col(8); mz = data.col(9);
	qx = data.col(10); qy = data.col(11); qz = data.col(12); qw = data.col(13);
	ex = data.col(14); ey = data.col(15); ez = data.col(16);
	rx = data.col(14) * M_DEG2RAD; ry = data.col(15) * M_DEG2RAD; rz = data.col(16) * M_DEG2RAD;
	dDistances = data.col(17); dYaws = data.col(18); dXs = data.col(19); dYs = data.col(20);

	inputs = join_horiz(inputs,ax); inputs = join_horiz(inputs,ay);
	inputs = join_horiz(inputs,az);
	// inputs = join_horiz(inputs,rx); inputs = join_horiz(inputs,ry);
	// inputs = join_horiz(inputs,rz);
	inputs = join_horiz(inputs,gx); inputs = join_horiz(inputs,gy);
	inputs = join_horiz(inputs,gz);

	initials.push_back(0);
	initials.push_back(0);
	initials.push_back(0);

	initials.push_back(0);
	initials.push_back(0);
	initials.push_back(0);
	initials.push_back(1);

	// initials.push_back(as_scalar(qx.row(0)));
	// initials.push_back(as_scalar(qy.row(0)));
	// initials.push_back(as_scalar(qz.row(0)));
	// initials.push_back(as_scalar(qw.row(0)));

	isam.init(initials);


	int ind0 = 0, indf = 0, zerCount = 0, eps = 10, steps = 0;

	// find the indices for when we first start moving and when we finally stop
	for(int t = 1;t<data.n_rows;t++){
		float dtraveled = as_scalar(dDistances.row(t));
		if(dtraveled > 0) flag_moved = 1;
		else if(dtraveled == 0  && !flag_moved)	ind0++;

		if(dtraveled == 0 && flag_moved){
			zerCount++;
			if(zerCount == eps){
				indf = t - eps;
			}
		}
	 	if(dtraveled != 0 && flag_moved) zerCount = 0;
	}


	fmat avg_offsets = inputs.rows(1,ind0);
	avg_offsets = mean(avg_offsets,0).t();
	avg_offsets.t().print("Averaged Offsets: ");
	offsets = avg_offsets;
	// offsets = inputs.row(1).t();

	flag_moved = 0;
	steps = 0;

	for(int t = ind0;t<=indf;t++){
		fmat imui = inputs.row(t);

		if(t == 0){
			dt = as_scalar(times.row(t));
			// printf("First Data Entry Time: %.4f\r\n\r\n",dt);
		}else
			dt = as_scalar(times.row(t) - times.row(t-1));

		float dV = as_scalar(dDistances.row(t)) / (1*dt);


		float timei = as_scalar(times.row(t));
		float dtraveled = as_scalar(dDistances.row(t));
		float dYaw = as_scalar(dYaws.row(t));
		float dX = as_scalar(dXs.row(t));
		float dY = as_scalar(dYs.row(t));
		// printf("t, Δdistance (m), ΔYaw (rad): %.5f, %.8f,  %.8f\n\r", d[0], d[1], d[2]);

		fmat in_cor = trans(imui)-offsets;
		in_cor.t().print("Corrected Outputs: ");

		if(dtraveled > 0)
			flag_moved = 1;

		if(dtraveled == 0 && !flag_moved){
			cout << "Data Skipped...." << endl;
			continue;
		}else{
			isam.update_odometry(dt, dtraveled, dYaw, dX, dY);
			isam.update_imu(dt, in_cor);
		}
	}
	isam.update();

	// isam.save(output_filename);
	fmat res = isam.convert_current_estimate();
	res.save(output_filename, csv_ascii);
	printf("t0, tf, y(0), y(f): %d, %d, %.3f, %.3f\r\n", ind0, indf,as_scalar(times.row(ind0)), as_scalar(times.row(indf)));
	avg_offsets.t().print("Averaged Offsets: ");

	return 0;
}

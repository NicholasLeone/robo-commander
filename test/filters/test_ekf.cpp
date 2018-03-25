// #define Nsta 2     // # state values:
// #define Mobs 3     // # measurements:

#include <unistd.h>
#include <iostream>
#include <fstream>

#include "ekf.h"
#include "utils/utils.h"
// #include <TinyEKF.h>

using namespace std;

int main(int argc, char *argv[]){

	int n;
	vector<float> initials;
	fmat outputs;
	string csv;
	string output_filename = "/home/hunter/devel/robo-dev/data/fusion/output/ekf_outputs.csv";

	if (argc < 2) {
		printf("Using Straight Line (~1m) .csv data file...\n");
		// csv = "/home/hunter/devel/robo-dev/data/fusion/1-0859m_straight_slow.csv";
		// csv = "/home/hunter/Documents/Swanson/1m_straight_slow.csv";
		csv = "/home/hunter/devel/robo-dev/data/fusion/ccw_turn_slow.csv";
	} else {
		csv = argv[1];
	}

	// FILE* fp_out = fopen(output_filename.c_str(), "w+");
  	// fprintf(fp_out, "Time(s),x(m),y(m),yaw(rad),v(m/s),w(rad/s),\n\r");

	fmat times, qx, qy, qz, qw, ax, ay, az, gx, gy, gz, mx, my, mz, ex, ey, ez, dDistances, dYaws;
     fmat data = csv_to_matrix(csv);

	times = data.col(0);
	ax = data.col(1); ay = data.col(2); az = data.col(3);
	gx = data.col(4); gy = data.col(5); gz = data.col(6);
	mx = data.col(7); my = data.col(8); mz = data.col(9);
	qx = data.col(10); qy = data.col(11); qz = data.col(12); qw = data.col(13);
	ex = data.col(14); ey = data.col(15); ez = data.col(16);
	dDistances = data.col(17); dYaws = data.col(18);

	initials.push_back(0);
	initials.push_back(0);
	initials.push_back(as_scalar(ez.row(0))*M_DEG2RAD);
	initials.push_back(as_scalar(dDistances.row(0)));
	initials.push_back(as_scalar(dYaws.row(0)));

// #ifdef DEBUG_VERBOSE
// 	testData.print("Input Data");
// 	//dataOut.print();
// 	cout << "Number of Inputs: " << testData.n_rows << " " << numData << endl;
// 	xInit.print("Initial States");
// #endif

	EKF ekf(5, 3);
	ekf.init(initials);

	for(int t = 1;t<data.n_rows;t++){
		fmat obs,row;
		float tk,xk,yk,yawk,vk,wk;
		float dt = as_scalar(times.row(t) - times.row(t-1));

		float wi = as_scalar(gz.row(t));
		float dV = as_scalar(dDistances.row(t)) / dt;
		float dW = as_scalar(dYaws.row(t)) / dt;

		obs << dV << endr
	 	    << dW << endr
	 	    << wi << endr;

		ekf.predict(dt);
		ekf.update(obs);

		tk = as_scalar(times.row(t));
		xk = as_scalar(ekf.xhat.row(0));
		yk = as_scalar(ekf.xhat.row(1));
		yawk = as_scalar(ekf.xhat.row(2));
		vk = as_scalar(ekf.xhat.row(3));
		wk = as_scalar(ekf.xhat.row(4));

		row << tk << xk << yk << yawk << vk << wk << endr;
		outputs = join_vert(outputs,row);

		// printf("t, Δdistance (m), ΔYaw (rad): %.5f, %.8f,  %.8f\n\r", d[0], d[1], d[2]);
	}

	//dataOut.print("Estimates");
	outputs.save(output_filename, csv_ascii);
	// cout << " Estimation Completed in " << ekf.steps << "! " << endl;
    	return 0;

}

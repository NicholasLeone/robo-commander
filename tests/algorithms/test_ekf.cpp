#include <unistd.h>
#include <iostream>
#include <fstream>

#include "algorithms/ekf.h"
#include "utilities/utils.h"

using namespace std;

int main(int argc, char *argv[]){

	int n = 21; int m = 12;
	float xf = 1.0859;
	float x0 = 0, y0 = 0, z0 = 0.15, q;
	fmat times, qx, qy, qz, qw, ax, ay, az, gx, gy, gz, mx, my, mz, ex, ey, ez, dDistances, dYaws, dXs, dYs;
	fmat outputs, inputsI, inputs, R, Q, offsets, zer;
	fmat rx, ry, rz; // Euler angles converted into radians
	string csv;
	string output_filename = "/home/hunter/devel/robo-dev/data/fusion/output/ekf_outputs.csv";
	string config_path = "";
	zer << 0 << endr;

	if (argc < 2) {
		// printf("Using Straight Line (~1m) .csv data file...\n");
		// csv = "/home/hunter/devel/robo-dev/data/fusion/1-0859m_straight_slow.csv";
		csv = "/home/hunter/devel/robo-dev/data/fusion/1-0859m_straight_slow.csv";
		// csv = "/home/hunter/devel/robo-dev/data/fusion/ccw_turn_slow.csv";
		// csv = "/home/hunter/devel/robo-dev/data/fusion/crude_1m_squareV3.csv";
	} else {
		csv = argv[1];
	}

	// Retrieve Data from file
	fmat data = csv_to_matrix(csv);
	// cout << "Csv Data Size: " << size(data) << endl;

	// Extract data into seperate variables
	times = data.col(0);
	ax = data.col(1); ay = data.col(2); az = data.col(3);
	gx = data.col(4); gy = data.col(5); gz = data.col(6);
	mx = data.col(7); my = data.col(8); mz = data.col(9);
	qx = data.col(10); qy = data.col(11); qz = data.col(12); qw = data.col(13);
	ex = data.col(14); ey = data.col(15); ez = data.col(16);
	rx = data.col(14) * M_DEG2RAD; ry = data.col(15) * M_DEG2RAD; rz = data.col(16) * M_DEG2RAD;
	dDistances = data.col(17); dYaws = data.col(18); dXs = data.col(19); dYs = data.col(20);

	// dataOut.print();
	// cout << "Number of Inputs: " << testData.n_rows << " " << numData << endl;

	// Initialize EKF
	EKF ekf(n, m);
	// ekf.print_stats();

	// Initial conditions: (x0, y0, z0, yaw0)
	vector<float> initials{x0, y0, z0, as_scalar(rz.row(0))};

	std::map<std::string, float> variables;
     LoadInitialVariables("/home/hunter/devel/robo-dev/config/filters/ekf.config", variables);

	float r1,r2,r3,r4,r5,r6,r7,r8,r9,r10,r11,r12;
     r1 = (float) variables["r1"];
     r2 = (float) variables["r2"];
     r3 = (float) variables["r3"];
     r4 = (float) variables["r4"];
     r5 = (float) variables["r5"];
     r6 = (float) variables["r6"];
     r7 = (float) variables["r7"];
     r8 = (float) variables["r8"];
     r9 = (float) variables["r9"];
     r10 = (float) variables["r10"];
     r11 = (float) variables["r11"];
     r12 = (float) variables["r12"];
     q = (float) variables["q"];

	R << r1	// xdot encoder
	  << r2	// ydot encoder
	  << r3	// xddot imu
	  << r4	// yddot imu
	  << r5 	// zddot imu
	  << r6	// roll imu
	  << r7	// pitch imu
	  << r8	// yaw imu
	  << r9	// roll dot imu
	  << r10	// pitch dot imu
	  << r11	// yawdot imu
	  << r12	// yawdot encoder
	  << endr;

	R.print("R: ");
	R = diagmat(R);
	Q = diagmat(ones<fmat>(n,1) * q);

	ekf.init(initials);
	ekf.set_R(R);
	ekf.set_Q(Q);

	// ekf.print_internals();
	// sleep(1);


	float tk,xhatk,yhatk,yawhatk,vhatk,whatk;
	float dt;

	// Concatenate Select data for easier input
	inputsI = join_horiz(inputsI,ax); inputsI = join_horiz(inputsI,ay);
	inputsI = join_horiz(inputsI,az); inputsI = join_horiz(inputsI,rx);
	inputsI = join_horiz(inputsI,ry); inputsI = join_horiz(inputsI,rz);
	inputsI = join_horiz(inputsI,gx); inputsI = join_horiz(inputsI,gy);
	inputsI = join_horiz(inputsI,gz);

	// offsets = inputsI.row(1);
	offsets = join_horiz(offsets,zer); offsets = join_horiz(offsets,zer);
	offsets = join_horiz(offsets, inputsI.row(1));
	offsets = join_horiz(offsets,zer);

	offsets.print("Offsets: ");
	offsets = offsets.t();
	sleep(1);


	for(int t = 1;t<data.n_rows;t++){
		fmat obs,row;
		if(t == 0){
			dt = as_scalar(times.row(t));
			// printf("First Data Entry Time: %.4f\r\n\r\n",dt);
		}else
			dt = as_scalar(times.row(t) - times.row(t-1));

		float dV = as_scalar(dDistances.row(t)) / (1*dt);

		fmat _xdot = dXs.row(t) / (1*dt);
		fmat _ydot = dYs.row(t) / (1*dt);
		fmat _yawdot = dYaws.row(t) / (-1*dt);

		float xdot = as_scalar(_xdot);
		float ydot = as_scalar(_ydot);
		float yawdot = as_scalar(_yawdot);

		obs = join_horiz(obs,_xdot); 			obs = join_horiz(obs,_ydot);
		obs = join_horiz(obs,inputsI.row(t)); 	obs = join_horiz(obs,_yawdot);

		// obs << xdot << ydot << inputsI.row(t) << yawdot << endr;
		obs = trans(obs);
		// cout << "Observation Size: " << size(obs) << endl;
		// obs.t().print("Observations (transposed): ");

		// obs << dV << endr
	 	//     << yawdot << endr
	 	//     << wi << endr;

		// tk = as_scalar(times.row(t));
		// xk = as_scalar(ekf.xhat.row(0));
		// yk = as_scalar(ekf.xhat.row(1));
		// yawk = as_scalar(ekf.xhat.row(2));
		// vk = as_scalar(ekf.xhat.row(3));
		// wk = as_scalar(ekf.xhat.row(4));

		ekf.predict(dt);
		ekf.update(obs-offsets);

		tk = as_scalar(times.row(t));
		xhatk = as_scalar(ekf.xhat.row(0));
		yhatk = as_scalar(ekf.xhat.row(1));
		yawhatk = as_scalar(ekf.xhat.row(14));
		vhatk = as_scalar(ekf.xhat.row(3));
		whatk = as_scalar(ekf.xhat.row(17));

		row << tk << xhatk << yhatk << yawhatk << vhatk << whatk << endr;
		outputs = join_vert(outputs,row);

		// printf("t, Δdistance (m), ΔYaw (rad): %.5f, %.8f,  %.8f\n\r", d[0], d[1], d[2]);
	}

	cout << "\r\nFinal Position Error: " << (xhatk - xf) << endl;
	//dataOut.print("Estimates");
	outputs.save(output_filename, csv_ascii);
	// cout << " Estimation Completed in " << ekf.steps << "! " << endl;
    	return 0;
}


// FILE* fp_out = fopen(output_filename.c_str(), "w+");
// fprintf(fp_out, "Time(s),x(m),y(m),yaw(rad),v(m/s),w(rad/s),\n\r");

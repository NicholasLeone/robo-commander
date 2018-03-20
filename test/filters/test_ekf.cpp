// #define Nsta 2     // # state values:
// #define Mobs 3     // # measurements:

#include <unistd.h>
#include <iostream>

#include "ekf.h"
#include "utils/utils.h"
// #include <TinyEKF.h>

using namespace std;


int numUpdates = 0;
int flag_exit = 0;

void my_handler(int s){
	flag_exit = 1;
	printf("DONE\r\n");
}

void up_handler(int s){
	numUpdates++;
	printf("VARIABLES UPDATED\r\n");
}

int main(){

	// VARIABLES
	int i = 0;
	float dt = 1;
	int n;
	// fmat testData, xInit, pInit, xhat, F, H, Q, R, sensorData, dataOut;

	string file = "/home/hunter/Documents/Data/Swanson/1m_straight_slow.csv";
	vector<vector<float>> entries = csv_extract_columns(file.c_str());
	vector<float> entry = entries.at(0);

	int nData = entries.size();
	int nVar = entry.size();
	printf("Test Data Size: %d, %d\n\r",nData,nVar);
// 	testData.load(file, csv_ascii);
// 	dataOut = zeros<fmat>(testData.n_rows, testData.n_cols);
//
// 	float initNoise = as_scalar(testData.row(0))*as_scalar(testData.row(0));
//
// 	xInit << 0 << endr
// 		 << 0 << endr;
//
// 	pInit << 0 << 0 << endr
// 		 << 0 << initNoise << endr;
//
// 	F << 1 << 0 << endr
// 	  << 0 << 1 << endr;
//
// 	H << 0 << 1 << endr;
//
// 	Q << 0 << 0 << endr
// 	  << 0 << 1 << endr;
//
// 	R << 0.01 << endr;
//
// #ifdef DEBUG_VERBOSE
// 	testData.print("Input Data");
// 	//dataOut.print();
// 	cout << "Number of Inputs: " << testData.n_rows << " " << numData << endl;
// 	xInit.print("Initial States");
// #endif

		EKF ekf();

		for(int j=0;j<nData;j++){

// 			float recorded = as_scalar(testData.row(j));
//
// 			sensorData << recorded << endr;
//
// 			// Update Sensors
// 			ekf.updateObservations(sensorData);
//
// 			// Predict States
// 			ekf.predict();
//
// 			// Check Predicted Value
// 			fmat tmpData = ekf.params_pred.x;
// 			float estimates = as_scalar(tmpData.row(1));
//
// 			cout << "Recorded Data, Estimated Data: " << recorded << ", 	" << estimates << endl;
//
// 			dataOut.row(j) = estimates;
//
// #ifdef DEBUG_VERBOSE
//
// #endif
			// cout << "Step, Prediction: " << j << ",		" << ekf.params_pred.x << endl;
		}


	//dataOut.print("Estimates");
	// dataOut.save("ekf_output.csv", csv_ascii);
	// cout << " Estimation Completed in " << ekf.steps << "! " << endl;
    	return 0;

}


/** TO COMPILE:

	g++ -w test_ekf.cpp ekf.cpp -o TestEkf -lpigpiod_if2 -larmadillo -Wall -pthread

*/

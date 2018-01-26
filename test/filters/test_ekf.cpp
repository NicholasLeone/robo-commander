#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <string.h>
#include <iostream>
#include <pthread.h>
#include <math.h>
#include <fstream>
#include <istream>
#include <signal.h>
#include <map>

// #include <pigpiod_if2.h>
#include <armadillo>
#include "ekf.h"

//#define DEBUG_VERBOSE
#define DEBUG_EKF

using namespace std;
using namespace arma;

char *optHost = NULL; // Use NULL when working on localhost
char *optPort = NULL; // Use NULL when working on localhost

static int pi;
static int numUpdates = 0;

int flag_exit = 0;
static ofstream myFile;

void my_handler(int s){

	myFile.close();
	// pigpio_stop(pi);
	flag_exit = 1;

	printf("DONE\r\n");
}


void up_handler(int s){
	numUpdates++;

	//std::map<std::string, float> variables;
    //LoadInitialVariables("config.txt", variables);

/**
    targetVel = (double) variables["target"];
    kp = (double) variables["kp"];
    ki = (double) variables["ki"];
    kd = (double) variables["kd"];
*/
	printf("VARIABLES UPDATED\r\n");
	//printf("TARGET SPEED, Kp, Ki, Kd: %.2f		%.2f 		%.2f 		%.2f \r\n",targetVel,kp,ki,kd);
}

void* sensorThreadFun(void* dump){

    double dt = 0.01;
    double dt_u = (dt) * 1000000;

    while(1){

    }

}

int countLines(const string &fileName){

	int numLines = 0;
	string tmpline;

	ifstream myfile(fileName.c_str());
	while(getline(myfile, tmpline)){++numLines;}

	return numLines;
}

int main(){

    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);


    struct sigaction sigUpHandler;
    sigUpHandler.sa_handler = up_handler;
    sigemptyset(&sigUpHandler.sa_mask);
    sigUpHandler.sa_flags = 0;
    sigaction(SIGTSTP, &sigUpHandler, NULL);

	// VARIABLES
	int i = 0;
	float dt = 1;
	int numData = countLines("sineData.csv");
	fmat testData, xInit, pInit, xhat, F, H, Q, R, sensorData, dataOut;

	testData.load("sineData.csv", csv_ascii);
	dataOut = zeros<fmat>(testData.n_rows, testData.n_cols);

	float initNoise = as_scalar(testData.row(0))*as_scalar(testData.row(0));

	xInit << 0 << endr
		 << 0 << endr;

	pInit << 0 << 0 << endr
		 << 0 << initNoise << endr;

	F << 1 << 0 << endr
	  << 0 << 1 << endr;

	H << 0 << 1 << endr;

	Q << 0 << 0 << endr
	  << 0 << 1 << endr;

	R << 0.01 << endr;

#ifdef DEBUG_VERBOSE
	testData.print("Input Data");
	//dataOut.print();
	cout << "Number of Inputs: " << testData.n_rows << " " << numData << endl;
	xInit.print("Initial States");
#endif

	// pi = pigpio_start(optHost, optPort); /* Connect to Pi. */

    pthread_t threader;
	pthread_create(&threader, NULL, sensorThreadFun, NULL);

    // if(pi >= 0){

		EKF ekf(xInit, pInit, F,H,Q,R, dt);

		/**
        myFile.open("ekf.csv");
        myFile << "Iteration,Speed,PWM\n";
		*/

        while(numUpdates<=0){}

		for(int j=0;j<=numData-1;j++){

			float recorded = as_scalar(testData.row(j));

			sensorData << recorded << endr;

			// Update Sensors
			ekf.updateObservations(sensorData);

			// Predict States
			ekf.predict();

			// Check Predicted Value
			fmat tmpData = ekf.params_pred.x;
			float estimates = as_scalar(tmpData.row(1));
			
			cout << "Recorded Data, Estimated Data: " << recorded << ", 	" << estimates << endl;

			dataOut.row(j) = estimates;

#ifdef DEBUG_VERBOSE

#endif
			// cout << "Step, Prediction: " << j << ",		" << ekf.params_pred.x << endl;
		}



		/**
        while(1){
            i = i + 1;

            if(flag_exit == 1)
                break;

#ifdef DEBUG_VERBOSE
            //cout << "Iteration: " << i << "		" << endl;
            //myFile << i << "," << curSpeed << "," << pwm << endl;
#endif
        }*/


		//while(flag_exit != 1){}

    // }

	//dataOut.print("Estimates");
	dataOut.save("ekf_output.csv", csv_ascii);
	cout << " Estimation Completed in " << ekf.steps << "! " << endl;
    	return 0;

}


/** TO COMPILE:

	g++ -w test_ekf.cpp ekf.cpp -o TestEkf -lpigpiod_if2 -larmadillo -Wall -pthread

*/

#include <unistd.h>
#include <iostream>

#include "filters/isam.h"
#include "utils/utils.h"

using namespace std;
using namespace arma;

int main(){

	// VARIABLES
	int i = 0;
	float d[3];
	int n;

	string csv = "/home/hunter/Documents/Data/Swanson/1m_straight_slow.csv";
     fmat data = csv_to_matrix(csv);

	d[0] = data(:,11).as_scalar();
	d[1] = data(:,12).as_scalar();
	d[2] = data(:,13).as_scalar();

	iSAM2 isam;

	isam.update_odometry(d);


    	return 0;

}

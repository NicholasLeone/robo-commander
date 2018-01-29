/*
 * mb7047.cpp
 */

/** TO COMPILE:

	g++ sensorHandler.cpp mb7047.cpp -o TestSONAR

*/


// Standard C++ header files
#include <iostream>
#include "mb7047.h"

// Standard C header files
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stropts.h>
#include <errno.h>
#include <time.h>
using namespace std;

//I2C Device Definitions

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* NAME:
*
* DESCRIPTION:
*
* INPUT:
*
* OUTPUT:
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
mb7047::mb7047(int bus, int address)
{

	//Device bus and address
	I2CBus = bus;
	I2CAddress = address;


	numberBytes = MB_I2C_BUFFER;

	this->mb7047_Open();

//	readFullSensorState();
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* NAME:
*
* DESCRIPTION:
*
* INPUT:
*
* OUTPUT:
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
mb7047::~mb7047( void )
{
	close(file);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* NAME:
*
* DESCRIPTION:
*
* INPUT:
*
* OUTPUT:
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void mb7047::mb7047_Open( void )
{

	snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);

	//Open a connection to the i2c bus.
	//Note this requires read/write access to the /dev/i2c-x file for a given user.
	if ((file = open(namebuf, O_RDWR)) < 0){
			//cout << "Failed to open MB7047 Sonar on " << namebuf << " I2C Bus." << endl;
			//cout << "Ensure the user has the correct permissions for the I2C Bus." << endl;
		//printf("AIR: Open Error.");
	}

}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* NAME:
*
* DESCRIPTION:
*
* INPUT:
*
* OUTPUT:
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
float mb7047::readFullSensorState()
{

///////////////////////////////////////////////////////////////////
// This subsection of code establishes a connection with the device,
// reads the data into a buffer using the linux/i2c-dev functions.
///////////////////////////////////////////////////////////////////

	//Dim file pointer and assign the i2c bus number to the string file
	numberBytes =10;

	//Access the specific slave address for the desired device
	if (ioctl(file, I2C_SLAVE, I2CAddress) < 0){
			//cout << "I2C_SLAVE address " << I2CAddress << " failed..." << endl;
			return(2);
	}

	/*Send the device specific start bit to the slave address.
	This initiates the data transfer b/w slave and master devices*/
	buf[0] = 224;
	buf[1] = 81;
 	if(write(file, buf, 1) !=1){
		//cout << "Failed to Reset Address in readFullSensorState() " << endl;
		return(3);
	}
	usleep(100000);

	/*Dim dataBuffer for the data coming in from the slave device*/

	char readBuf[1];
	readBuf[0] = 225;
	if(write(file, readBuf, 1) !=1){
		//cout << "Failed to Reset Address in readFullSensorState() " << endl;
		return(3);
	}
	usleep(100000);
	//Read in the data stream to the buffer. Upon success close the file; otherwise, stream error.
	bytesRead = read(file, dataBuffer, numberBytes);

	cout << "Bytes Read: " << bytesRead << endl;

	if (bytesRead == -1){
		//cout << "Failure to read Byte Stream in readFullSensorState()" << endl;
		return(4);
	}



	// if (dataBuffer[0] >> 6 == 1){
	// 	//cout << "MAJOR FAILURE: Device is in command mode. See data sheet for details." << endl;
	// 	return(5);
	//
	// } else if(dataBuffer[0] >> 6 == 2) {
	// 	//cout << "MAJOR FAILURE: Device is returning stale data. See data sheet for details." << endl;
	// 	return(6);
	// } else if(dataBuffer[0] >> 6 == 3) {
	// 	//cout << "MAJOR FAILURE: Device is diagnostic mode. See data sheet for details." << endl;
	// 	return(7);
	// }


	//Distance reading
	//Strip the status bits from the first byte
	dataBuffer[0] = (dataBuffer[0] << 2) >> 2;
	cout<<"DATA BUFFER :" <<dataBuffer[0]<< "	DATA BUFFER: "<<dataBuffer[1]<< "	DATA BUFFER: "<<dataBuffer[2] <<endl;

	value = (float)( ((int) dataBuffer[0] << 8) + dataBuffer[1]);
	//int val = (int)dataBuffer[0] << 8 | dataBuffer[1]
	return value;

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * *
*
* NAME:
*
* DESCRIPTION:
*
* INPUT:
*
* OUTPUT:
*
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
float mb7047 :: getDistance()
{


	//Query sensor data and assign the pressure its value based on unit input
	distance = readFullSensorState();
	cout<<"Distance : "<<distance<<endl;
	return distance;

}

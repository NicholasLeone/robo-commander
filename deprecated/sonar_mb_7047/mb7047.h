

#ifndef MB7047_H_
#define MB7047_H_
#define MB7047_I2C_BUFFER 0x04

#define MAX_BUS 64
#define MB_I2C_BUFFER 0x04
class mb7047 {
private:

	int I2CAddress, I2CBus ;

	char dataBuffer[10];
	char namebuf[MAX_BUS];
	char buf[2];

	int numberBytes;
	int bytesRead;
	int file;

public:
	//Constructor function for the C++ class
	mb7047(int bus, int address);

	//Calls the buffer to
	float readFullSensorState();
	float getDistance();
	float value;
	float distance;
	//Function destuctor
	~mb7047( void );
	void mb7047_Open( void );

};

#endif /* MB7047_H_ */

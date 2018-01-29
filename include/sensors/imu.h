#ifndef IMU_H_
#define IMU_H_

class IMU{

private:
     int address;
     int port;
     Sim_Msg_IMUData data;

public:
	IMU();
	~IMU();

     void print();

};


#endif /* IMU_H_*/

#ifndef SERIAL_H_
#define SERIAL_H_

#include <boost/asio.hpp>

using namespace std;
using namespace boost;

class SerialDev{

private:
     char read_msg_[512];
     asio::io_service io;
     asio::serial_port port;
public:
     bool active;

     SerialDev(const char* add, unsigned int baud);
     ~SerialDev();

     string readLine();
     void writeLine(string s);


};


#endif /* SERIAL_H_ */

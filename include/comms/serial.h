#ifndef SERIAL_H_
#define SERIAL_H_

#include <unistd.h>
#include <string>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

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

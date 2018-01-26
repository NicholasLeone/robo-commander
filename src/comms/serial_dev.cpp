#include "string.h"
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "serial_dev.h"

using namespace std;
using namespace boost;

SerialDev::SerialDev(const char* add, unsigned int baud): port(io){

     port.open(add);
     port.set_option(asio::serial_port_base::baud_rate(baud));
     port.set_option(asio::serial_port_base::character_size(8));
     port.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
     port.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
     port.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
     active = port.is_open();
}

SerialDev::~SerialDev(){
     port.cancel();
     port.close();
     cout << "SerialDev Closed" << endl;
}

string SerialDev::readLine(){
     char c;
     string result;

     while(1){
          asio::read(port, asio::buffer(&c, 1));
          switch(c){
               case '\r':
                    break;
               case '\n':
                    return result;
               default:
                    result += c;
          }
     }

}

void SerialDev::writeLine(string s){

     asio::write(port, asio::buffer(s.c_str(), s.size()));
}

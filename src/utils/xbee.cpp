/** TO COMPILE:

     g++ -w xbee.cpp -o TestXbee

*/
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

int main() {
    char byte;
    int fd = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK);
    write(fd, "X", 1);
    //ssize_t size = read(fd, &byte, 1);
    printf("Read byte %c\n", byte);
    return 0;
}

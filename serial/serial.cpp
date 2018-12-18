#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <errno.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>

#include "serial.h"

static int fd = 0;
static std::string inbuf;

void error_message(const char *message, int error){
    fprintf(stderr, "%s: %s\n", message, strerror(error));
    exit(1);
}

int set_interface_attribs(int fd, int speed, int parity){
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0){
        error_message("error %d from tcgetattr", errno);
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag =(tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |=(CLOCAL | CREAD);// ignore modem controls,
    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if(tcsetattr(fd, TCSANOW, &tty) != 0){
        error_message("error %d from tcsetattr", errno);
    }
    return 0;
}

void set_blocking(int fd, int should_block){
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if(tcgetattr(fd, &tty) != 0){
        error_message("error %d from tggetattr", errno);
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if(tcsetattr(fd, TCSANOW, &tty) != 0){
        error_message("error %d setting term attributes", errno);
    }
}

//B115200, B230400, B9600, B19200, B38400, B57600, B1200, B2400, B4800
void openSerial(const char *port, int baud){
    fd = open(port, O_RDWR | O_NOCTTY | O_SYNC);
    if(fd < 0){
        error_message("error opening device", errno);
    }

    set_interface_attribs(fd, baud, 0);  // set speed to 115,200 bps, 8n1(no parity)
    set_blocking(fd, 1);                // set blocking

    tcflush(fd, TCIFLUSH);
}

void closeSerial(){
    close(fd);
}

void serialWrite(const char *str, int len){
    write(fd, str, len);
}

int serialRead(char *buf, int bufSize){
    return read(fd, buf, bufSize);
}

void removeNull(char *buf, int len){
    for(int i = 0; i < len; ++i){
        if(buf[i] == 0){
            buf[i] = ' ';
        }
    }
    buf[len - 1] = 0;
}

std::string serialReadLine(){
    static char buf[128];
    size_t index = inbuf.find('\n');

    //while we haven't found a line ending
    while(index == std::string::npos){
        int len = serialRead(buf, 128);
        removeNull(buf, len);
        inbuf.append(buf);
        index = inbuf.find('\n');
    }

    std::string line = inbuf.substr(0, index);
    inbuf.erase(0, index + 1);
    //reclaim some memory
    inbuf.shrink_to_fit();

    return line;
}

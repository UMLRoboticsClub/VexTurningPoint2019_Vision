#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>

int fd = 0;

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
    set_blocking(fd, 0);                // set no blocking
}

void closeSerial(){
    close(fd);
}

void serialWrite(const char *str, int len){
    write(fd, str, len);                  // send 7 character greeting
    //usleep((7 + 25) * 100);             // sleep enough to transmit the 7 plus
}

//void serialWriteChar(const char c){
//    fputc(c, fd);
//}

int serialRead(char *buf, int bufSize){
    return read(fd, buf, bufSize);
}

//int serialReadChar(){
//    return fgetc(fd);
//}

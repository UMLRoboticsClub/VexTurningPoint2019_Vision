#pragma once

#include <string>

void error_message(const char *message, int error);
int set_interface_attribs(int fd, int speed, int parity);
void set_blocking(int fd, int should_block);
//B115200, B230400, B9600, B19200, B38400, B57600, B1200, B2400, B480
void openSerial(const char *port, int baud);
void closeSerial();
void serialWrite(const char *str, int len);
int serialRead(char *buf, int bufSize);
std::string serialReadLine();

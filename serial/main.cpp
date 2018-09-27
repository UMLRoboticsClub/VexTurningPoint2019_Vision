
#include <iostream>
#include <string>

#include <csignal>
#include <cstring>

#include <termios.h>

#include "serial.h"

using std::cin;
using std::cout;
using std::endl;
using std::string;

char *serialPortName;
unsigned serialBaudRate;
const char *header = "zz ";

void output(){
    int headerLen = strlen(header);

    string input;
    while(cin){
        std::getline(cin, input); 
        if(strncmp(input.c_str(), header, headerLen) == 0){
            serialWrite(input.c_str(), input.size());
        } else {
            puts(input.c_str());
        }
    }
}

void input(){
    //does nothing
}

void setup(){
    cout << "using port " << serialPortName << " at " << serialBaudRate << " baud" << endl;

    //B115200, B230400, B9600, B19200, B38400, B57600, B1200, B2400, B4800
    switch(serialBaudRate){
        case 115200:
            serialBaudRate = B115200;
            break;
        case 9600:
            serialBaudRate = B9600;
            break;
    }
    
    openSerial(serialPortName, serialBaudRate);
}

int main(int argc, char **argv){
    if(argc < 3){
        cout << "missing arguments" << endl;
        cout << "suggested usage: ./serial /dev/ttyUSB0 115200" << endl;
        exit(1);
    }

    serialPortName = argv[1];
    serialBaudRate = strtol(argv[2], NULL, 10);

    setup();

    if(argc == 4 && strcmp(argv[3], "i") == 0){
        input();
    } else {
        output();
    }

    closeSerial();
    puts("EOF reached, bye");
}

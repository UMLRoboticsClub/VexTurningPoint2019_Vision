#include <iostream>
#include <string>

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>

#include <unistd.h>
#include <termios.h>
#include <signal.h>

#include "serial.h"

//#define DEBUG_OUTPUT

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::getline;

static char *serialPortName;
static unsigned serialBaudRate;
static const char *header = "zz ";

static fd_set readfds;
//seconds, microseconds
static struct timeval timeout { 0, 50000 };

static bool running = true;

//better header, needs testing
//#define STX 0x1
//#define SOT 0x2
//#define EOT 0x4
//const char *header = "STX, STX, SOT";

void checkInput(){
    static char buf[256]; 
    int len = serialRead(buf, 256);

    //error or zero bytes read
    if(len < 1){
        return;    
    }

    //for(int i = 0; i < len; ++i){
    //    buf[i] = buf[i + 5];
    //}

    cout << "[R]: " << buf << endl;
}

//is there data in stdin?
bool dataAvailable(){
    return select(1, &readfds, NULL, NULL, &timeout) > 0;
}

//read a line, if header exists, send it over serial
//otherwise print it
void readAndProcessData(){
    int headerLen = strlen(header);

    string input;
    while(running){
        while(dataAvailable()){
            getline(cin, input);

            //does the header exist?
            if(strncmp(input.c_str(), header, headerLen) == 0){
                serialWrite(input.c_str(), input.size());
                cout << "[S]: " << input << endl;
#ifdef DEBUG_OUTPUT
                cout << input << endl;
#endif
            } else {
                //if no header, it's a debug message, print it
                cout << "[V]: " << input << endl;
            }
        }
        //while(serialAvailable()){
        //    checkInput();
        //}
        checkInput(); //non-blocking
    }
}

void setup(){
    cout << "using port " << serialPortName << " at " << serialBaudRate << " baud" << endl;

    //options are: B115200, B230400, B9600, B19200, B38400, B57600, B1200, B2400, B4800
    switch(serialBaudRate){
        case 115200:
            serialBaudRate = B115200;
            break;
        case 9600:
            serialBaudRate = B9600;
            break;
    }

    openSerial(serialPortName, serialBaudRate);

    FD_ZERO(&readfds);
    FD_SET(0, &readfds);
}

void signalHandler(int){
    running = false;
}

int main(int argc, char **argv){
    if(argc < 3){
        cout << "missing arguments" << endl;
        cout << "suggested usage: ./serial /dev/ttyUSB0 115200" << endl;
        exit(1);
    }

    serialPortName = argv[1];
    serialBaudRate = strtol(argv[2], NULL, 10);

    signal(SIGINT, signalHandler);

    setup();
    readAndProcessData();

    closeSerial();
    puts("\nexiting, bye");
}

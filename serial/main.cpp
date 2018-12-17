#include <iostream>
#include <string>
#include <thread>
#include <mutex>

#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <cstring>

#include <signal.h>
#include <termios.h>

#include "serial.h"

//#define DEBUG_OUTPUT

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::getline;
using std::thread;

static char *serialPortName;
static unsigned serialBaudRate;
static const char *header = "zz ";

static std::mutex mutex;

//better header, needs testing
//#define STX 0x1
//#define SOT 0x2
//#define EOT 0x4
//const char *header = "STX, STX, SOT";

#define RST  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"

#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define FYEL(x) KYEL x RST
#define FBLU(x) KBLU x RST
#define FMAG(x) KMAG x RST
#define FCYN(x) KCYN x RST
#define FWHT(x) KWHT x RST

void signalHandler(int){
    closeSerial();
    puts("\nexiting, bye");
    exit(0);
}

void checkInput(){
    /*
    static char buf[512]; 
    int len = serialRead(buf, 512);

    if(len < 1){
        signalHandler(0);
    }

    for(int i = 0; i < len; ++i){
        char c = buf[i];
        if(c == 0){
            buf[i] = ' ';
        }
    }
    buf[len] = 0;

    mutex.lock();
    cout << "[R]: " << buf;
    fflush(stdout);
    mutex.unlock();
    */

    string line = serialReadLine();
    mutex.lock();
    cout << FRED("[R]: ") << line << endl;
    mutex.unlock();
}

//read a line, if header exists, send it over serial
//otherwise print it
void readAndProcessData(){
    int headerLen = strlen(header);
    string input;

    while(1){
        while(getline(cin, input)){
            input += '\n';
            mutex.lock();
            //does the header exist?
            if(strncmp(input.c_str(), header, headerLen) == 0){
                serialWrite(input.c_str(), input.size());
                cout << FGRN("[S]: ") << input << endl;
            } else {
                //if no header, it's a debug message, print it
                cout << FBLU("[V]: ") << input << endl;
            }
            mutex.unlock();
        }
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

    thread writeThread(readAndProcessData);
    writeThread.detach();

    while(1){
        checkInput();
    }
}

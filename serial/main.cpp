//TODO: add functionality to send data about team color

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
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
using namespace std::chrono_literals;

static char *serialPortName;
static unsigned serialBaudRate;
static const char *header = "zz ";
static int HEARTBEAT_THRESH = 2000;
//TODO: maybe adjust this later
static int STARTUP_THRESH = 5000;

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

typedef std::chrono::time_point<std::chrono::high_resolution_clock> timept;

timept now(){
    return std::chrono::high_resolution_clock::now();
}

int diff(const timept a, const timept b){
    return std::chrono::duration_cast<std::chrono::milliseconds>(a - b).count();
}

class {
    public:
        const timept getTime(){
            std::lock_guard<std::mutex> lock(mutex);
            return lastMsg;
        }

        void setTime(const timept &t){
            std::lock_guard<std::mutex> lock(mutex);
            lastMsg = t;
        }
    private:
        std::mutex mutex;
        timept lastMsg;
} LastMsg;


void signalHandler(int){
    closeSerial();
    puts("\nexiting, bye");
    exit(0);
}

void checkInput(){
    string line;

    while(true){
        line = serialReadLine();
        LastMsg.setTime(now());
        if(line.empty()){
            continue;
        }
        cout << FRED("[R]: ") << line << endl;
    }
}

//read a line, if header exists, send it over serial
//otherwise print it
void readAndProcessData(){
    const int headerLen = strlen(header);
    string input;

    while(true){
        while(getline(cin, input)){
            //does the header exist?
            if(strncmp(input.c_str(), header, headerLen) == 0){
                cout << FGRN("[S]: ") << input << endl;
                input += '\n';
                serialWrite(input.c_str(), input.size());
            } else {
                //if no header, it's a debug message, print it
                cout << FBLU("[V]: ") << input << endl;
            }
        }
        std::this_thread::sleep_for(20ms);
    }
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

    thread writeThread(readAndProcessData);
    writeThread.detach();

    //start checking 5 sec from now
    LastMsg.setTime(now() + std::chrono::milliseconds(STARTUP_THRESH));
    thread inputThread(checkInput);
    inputThread.detach();

    //check if messages are being received
    while(1){
        //cout << diff(now(), LastMsg.getTime()) << endl;
        if(diff(now(), LastMsg.getTime()) > HEARTBEAT_THRESH){
            cout << "Not receiving data, exiting" << endl;
            exit(1);
        }
        std::this_thread::sleep_for(200ms);
    }
}

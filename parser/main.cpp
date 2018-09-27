//#define DEBUG
#include <iostream>
#include <cstdio>
#include <string>
#include <cstring>
#include <vector>
#include <climits>
#include <utility>

#include "crc.h"

using std::cin;
using std::cout;
using std::endl;
using std::string;
using std::vector;

typedef std::pair<int, int> Point;

void parseInput(string &input, vector<Point> &targets){
    const char *buf = input.c_str();
#ifdef DEBUG
    cout << "input: " << buf << endl;
    cout << "size : " << input.size() << endl;
#endif
    int pktIndex = 0;

    //check header
    const char *header = "zz ";
    const int headerSize = strlen(header);

    if((int)input.size() < headerSize + 8){
#ifdef DEBUG
        cout << "packet messed up: too small" << endl;
#endif
        return;
    }

    while(isspace(buf[pktIndex + 1])) ++pktIndex; //skip spaces

    for(; pktIndex < headerSize; ++pktIndex){
        if(header[pktIndex] != buf[pktIndex]){
#ifdef DEBUG
            cout << "packet messed up: bad header" << endl;
#endif
            return;
        }
    }
#ifdef DEBUG
    cout << "packet header ok" << endl;
#endif

    while(isspace(buf[pktIndex + 1])) ++pktIndex; //skip spaces

    char *end;
    const char *afterHeader = buf + pktIndex - 1;
    //get size
    const int size = strtol(afterHeader, &end, 10);
    //digits of 'size'
    const int sizeSize = end - afterHeader;
    for(int i = 0; i < size; ++i){
        //get the rest of the numbers
        targets.emplace_back((int)strtol(end, &end, 10), (int)strtol(end, &end, 10));
#ifdef DEBUG
        cout << "point:" << '[' << targets.back().first << "," << targets.back().second << ']' << endl;
#endif
    }

    //cout << "size size: " << sizeSize << endl;

    const char *dataStart = buf + pktIndex + sizeSize;
    const int dataLen = end - buf - (dataStart - buf);

#ifdef DEBUG
    for(int i = 0; i < dataLen; ++i){
        cout << *(dataStart + i);
    }
    cout << endl;
#endif


    uint32_t repcrc = strtol(end, &end, 10);
    uint32_t gencrc = crc32buf((char*)dataStart, dataLen);
#ifdef DEBUG
    cout << "reported crc: " << repcrc << endl;
    cout << "generated crc: " << gencrc << endl;
#endif
    if(repcrc != gencrc) {
#ifdef DEBUG
        cout << "bad crc" << endl;
        cout << endl;
#endif
        targets.clear();
        return;
    }
#ifdef DEBUG
    cout << "CRCs match!" << endl;
    cout << endl;
#endif
}

void doThing(vector<Point> &targets){
    //targets.size();
    for(auto &a : targets){
        cout << "point:" << '[' << a.first << "," << a.second << ']' << endl;
    }
}

int main(){
    string input;
    while(cin){
        std::getline(cin, input);
        if(input == "") continue; //skip empty lines
        vector<Point> targets;
        parseInput(input, targets);
        doThing(targets);
    }
}
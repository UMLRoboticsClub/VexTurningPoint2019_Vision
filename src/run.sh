#!/bin/sh
#vision prints the serial packets interspersed with debug/status messages
#serial will only send the packets and will print the messages
./vision | ../serial/serial /dev/ttyUSB0 115200
#./vision | ../parser/parser

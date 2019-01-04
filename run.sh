#!/bin/sh

#vision prints the serial packets interspersed with debug/status messages
#serial will only send the packets and will print the messages

./vision/vision | ./serial/serial /dev/ttyACM1 115200

#./vision | tee /dev/tty | ../serial/serial /dev/ttyACM1 115200
#./vision | ../parser/parser

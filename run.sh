#!/bin/bash

#vision prints the serial packets interspersed with debug/status messages
#serial will only send the packets and will print the messages

#exits when serial fails
mkfifo fifo
./vision/vision > fifo &
./serial/serial /dev/ttyACM1 115200 < fifo

pkill vision
rm fifo

#basic
#./vision/vision | ./serial/serial /dev/ttyACM1 115200

#other options for testing
#./vision/vision | tee /dev/tty | /serial/serial /dev/ttyACM1 115200
#./vision/vision | /parser/parser

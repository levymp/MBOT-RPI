#!/bin/bash
source setenv.sh
./bin/timesync &> /dev/null &
./bin/rplidar_driver &> /dev/null &
./bin/motion_controller_new &> /dev/null &
./bin/botgui

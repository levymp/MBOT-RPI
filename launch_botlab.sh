#!/bin/bash
source setenv.sh
./bin/timesync &> /dev/null &
./bin/rplidar_driver &> /dev/null &
<<<<<<< HEAD
./bin/slam &> /dev/null &
=======
>>>>>>> 13cd5b4a6669c66409e296b611c3f6671d268d29
./bin/motion_controller_new &> /dev/null &
./bin/botgui

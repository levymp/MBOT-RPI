#! /usr/bin/ python3
import sys
import requests
import subprocess
import time
import signal
import os
from shutil import move
from pathlib import Path
from mbotdefs import BOTNAME
import os

_BASE_URL = 'https://api.mplevy.com/api/mbot/v1/'
_LOG_URL = _BASE_URL + 'log'


def post_log(botname, description, path):
    '''pass in path string'''
    # read file
    file_path = Path(path)
    
    # check if correct path
    if not file_path.is_file():
        return -1

    # open file
    file = open(file_path, 'rb')

    # create payload
    payload = {'logfile': file}
    param = {'name': botname, 'description': description}
    
    # post file
    r = requests.post(_LOG_URL, params=param, files=payload)
    
    # close file
    file.close()
    
    # check status code
    if r.status_code != 200: 
        print('FILE WAS NOT POSTED!')
        print(r.text)
        return -1
    else: 
        print('FILE POSTED!')
        return r.json()


def main():
    # start the timesync
    # p_time = subprocess.Popen(['./timesync'], cwd=os.path.abspath("../bin"), stdin=subprocess.PIPE)

    channels_to_watch = "ODOMETRY|CONTROLLER_PATH|MBOT_ENCODERS|MBOT_IMU|MBOT_STATE|MBOT_SETPOINTS|SLAM_POSE|TRUE_POSE|MBOT_MOTOR_COMMAND"
    # path to temporary file
    temp_file_path = '../static/log_temp.log'

    # delete temp file if it's still there
    if os.path.exists(temp_file_path):
        os.remove(temp_file_path)

    description = input('RUN DESCRIPTION: ')

    # start the logger
    if(len(sys.argv) > 1):
        p_logger = subprocess.Popen(['lcm-logger', temp_file_path, '-c', channels_to_watch, '-q', '-f'], stdin=subprocess.PIPE)
    else: 
        p_logger = subprocess.Popen(['lcm-logger', temp_file_path, '-q', '-f'], stdin=subprocess.PIPE)
    
    print('LOGGER STARTED!')

    # start looping until cancelled 
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            print('PUBLISHING LOG! (DON\'T PRESS ^C again)')
            # stop the log file
            p_logger.send_signal(signal.SIGINT)
            # post the log
            r = post_log(BOTNAME, description, temp_file_path)
            if not isinstance(r, int):
                print('POSTED FILE RunId : ' + str(r['runId']))
            else:
                print('UNABLE TO POST FILE... PUBLISH MANUALLY BEFORE NEXT RUN!')

            # exit
            exit(0)

if __name__ == "__main__":
    main()

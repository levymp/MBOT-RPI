#! /usr/bin/ python3
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
    p_time = subprocess.Popen(['./timesync'], cwd=os.path.abspath("../bin"), stdin=subprocess.PIPE)

    # get user input of what is going on
    description = input('RUN DESCRIPTION: ')

    # path to temporary file
    temp_file_path = '../static/log_temp.log'

    # delete temp file if it's still there
    if os.path.exists(temp_file_path):
        os.remove(temp_file_path)
    
    # start the logger
    p_logger = subprocess.Popen(['lcm-logger', temp_file_path, "-q"], stdin=subprocess.PIPE)
    
    print('LOGGER & TIME SYNC STARTED!')

    # start looping until cancelled 
    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            # stop the log file
            p_logger.send_signal(signal.SIGINT)
            # stop time 
            p_time.send_signal(signal.SIGINT)
            # post the log
            r = post_log(BOTNAME, description, '../static/log_temp.log')
            if not isinstance(r, int):
                print(r)

            # exit
            exit(0)

if __name__ == "__main__":
    main()

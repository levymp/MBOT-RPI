import requests
import subprocess
import time
import signal
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
    if r.status_code != 200: 
        print(r.text)
        return -1
    else: 
        return r.json()


def main():
    # start the timesync
    p_time = subprocess.Popen(['timesync'], cwd=os.path.abspath("../bin"))

    # start the logger
    p_logger = subprocess.Popen(['lcm-logger', "../static/log_test", "-i", "-q"])
    

    # stop the log file
    p_logger.send_signal(signal.SIGINT)

    # stop time 
    p_time.send_signal(signal.SIGINT)




if __name__ == '__main__':
    main()
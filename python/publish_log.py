import requests
import subprocess
import time
import signal
import os


# # start the timesync
# p_time = subprocess.Popen(['timesync'], cwd=os.path.abspath("../bin"))
# HOST = ""
# COMMAND = 
# # start mobile bot
# p_mobilebot = subprocess.Popen(["ssh", "%s" % HOST, COMMAND],

# )


# # start the logger
# p_logger = subprocess.Popen(['lcm-logger', "../static/log_test", "-i", "-q"])
# time.sleep(20)

# # stop the log file
# p_logger.send_signal(signal.SIGINT)

# # stop time 
# p_time.send_signal(signal.SIGINT)

def post_log(file_name):
    file = open('../static/' + file_name, 'rb')
    payload = {'logfile': file}
    url = 'https://api.mplevy.com/api/mbot/v1/log'

    r = requests.post(url, files=payload)

    print(r.status_code)
    print(r.json())
    
    file.close()


if __name__ == '__main__':
    post_log('first_log.log')
import requests
import subprocess
import time
import signal
import os


# start the timesync
p_time = subprocess.Popen(['timesync'], cwd=os.path.abspath("../bin"))
# HOST = ""
# COMMAND = 
# start mobile bot
# p_mobilebot = subprocess.Popen(["ssh", "%s" % HOST, COMMAND])

# start the logger
p_logger = subprocess.Popen(['lcm-logger', "../static/log_test", "-i", "-q"])
time.sleep(20)

# stop the log file
p_logger.send_signal(signal.SIGINT)

# stop time 
p_time.send_signal(signal.SIGINT)



# def post_log(file_name):
#     file = open('../static/first_log.log', 'rb')
#     # print(file)
#     payload = {'logfile': file}
#     url = 'http://127.0.0.1:8505/MBOT/v1/api/'
#     # r = requests.get('https://ip.picotrillion.com/api/geo/v1/ip')

#     r = requests.post(url, files=payload)

#     print(r.status_code)
#     print(r.json())
#     print(r.status_code)
#     # val = r.json()
#     file.close()
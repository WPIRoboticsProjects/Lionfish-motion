from pymavlink import mavutil
from multiprocessing import Process, Queue
import time
from sys import exit
import serial

# Pwm channel pins
# 0 - pitch
# 1 - roll
# 2 - up
# 3 - yaw
# 4 - forward
# 5 - lateral
# 6 - camera pan
# 7 - camera tilt
# 8 - lights 1 level

TURN_BUFFER = 2

def run(q):
    cont_run = True
    while cont_run:
        command = input("Command: ")
        print("Given command: " + command + "\n")



if __name__ == '__main__':
    # signal(SIGINT, handler)
    q = Queue()
    arduinoProcess = Process(target=run, args=(q,))
    arduinoProcess.start()
    while True:
        pass
    # time.sleep(5)
    # print("data return: ", end='')
    # val = q.get()
    # for i in range(q.qsize()):
    #     val = q.get()
    # print(val)
    #
    # main()
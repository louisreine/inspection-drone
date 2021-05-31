# coding=utf-8
from simple_pid import PID
from time import time
import pandas as pd

""" This code uses a PID to control the drone in speed. The PID takes as input the distance to the obstacle and 
gives a speed command (m/s) to drone. Their is a desired distance to reach and the command saturates at +- MAX_SPEED
"""

SAMPLE_TIME = 0.2  # the lower the sample time is, the preciser the convergence will be
MAX_SPEED = 2  # maximum speed for the drone in m/s
DESIRED_DISTANCE = 3   # desired distance entered by the user
CURRENT_DISTANCE = 10  # initial distance in the modelled system.
# To be replace with the distance read by the Lidar on the drone
LAST_ITERATION_TIME = time()
TIME_INIT = LAST_ITERATION_TIME
# Paramètres du PID
KP = 2
KI = 0.5
KD = 0.1


def syst(consigne):
    return consigne*SAMPLE_TIME


if __name__ == '__main__':
    pid = PID(KP, KI, KD, setpoint=1)
    pid.sample_time = SAMPLE_TIME  # Update every 0.01 seconds
    pid.output_limits = (-MAX_SPEED, MAX_SPEED)  # Output value will be between -2 and 2
    # List to record the distances history to then visualize it
    distance_list = [CURRENT_DISTANCE]
    time_list = [0]

    while len(distance_list) < 20/SAMPLE_TIME:
        if time()-LAST_ITERATION_TIME >= SAMPLE_TIME:
            consigne = pid(CURRENT_DISTANCE-DESIRED_DISTANCE)  # consigne correspond to the speed commanded to the drone
            CURRENT_DISTANCE += syst(consigne)  # To be replaced on the drone with the distance read by the Lidar
            distance_list.append(CURRENT_DISTANCE)
            LAST_ITERATION_TIME = time() # We update the time of the last iteration
            time_list.append(LAST_ITERATION_TIME - TIME_INIT)
            # print("consigne", consigne, "distance", CURRENT_DISTANCE)

    # We save the data to plot it in a csv file
    dict = {'distances':distance_list, 'time':time_list}
    df = pd.DataFrame(dict, columns=['time','distances'])
    df.to_csv('file1.csv', sep=';')
    # A file named "file1.csv" is created in the work directory


from Range_Sensors import *
import matplotlib.pyplot as plt

def obstacle_Detected(mySonar, myLidar, debug = False):
    """ Function that takes as input two objects of type sonar and lidar
    and that returns True if the distance they read is inferior to the critical distance defined for each of them
    The param debug has to be set to True if you want the data to be printed when uptdated"""
    read_serial = readSensorsLine()
    if mySonar.read_distance(read_serial) and debug:
        print(f"Sonar range: {S.get_distance()}")
    if myLidar.read_distance(read_serial) and debug:
        print(f"Lidar range: {L.get_distance()}")
    if mySonar.critical_Distance_Reached() or myLidar.critical_Distance_Reached():
        return True
    return False

def print_Logs(mySonar, myLidar):
    """ Function that takes as input two ojects of type sonar and lidar
    and prints their log, the history of the data they read """
    plt.figure()
    plt.title("Ranged mesured by the sonar and the lidar")
    plt.xlabel("Time in s")
    plt.ylabel("Distance in cm")
    mySonar.see_log()
    myLidar.see_log()
    plt.legend()
    plt.show()   

# -------- main ---------

# We start by initializing our 2 sensors.
# We can give as parameter the critical distance of the sensor (under which we detect an obstacle)
# By default, the critical distance is 50 cm
S = Sonar()
L = Lidar(40)

runningTime = 10 # Time in s for which we collect the data

while time.time() - S.startTime < runningTime:
    if obstacle_Detected(S, L):
        print("\\n TU VAS TE PRENDRE LE MUR !! \\n")

print_Logs(S, L)
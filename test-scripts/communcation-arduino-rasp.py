from Range_SensorsP2 import *
import RPi.GPIO as GPIO
# Select GPIO mode
GPIO.setmode(GPIO.BCM)
# Set buzzer - pin 23 as output
buzzer = 23
GPIO.setup(buzzer, GPIO.OUT)

def obstacle_Detected(mySonar, myLidar, use_lidar, use_sonar, debug=False, filtre = True):
    """ Function that takes as input two objects of type sonar and lidar
    and that returns True if the distance they read is inferior to the critical distance defined for each of them
    The param debug has to be set to True if you want the data to be printed when uptdated"""
    read_serial = readSensorsLine()
    if mySonar.read_distance(read_serial, filtrage = filtre) and debug:
        print "Sonar range:" + str(mySonar.get_distance())
    if myLidar.read_distance(read_serial, filtrage = filtre) and debug:
        print "Lidar range:" + str(myLidar.get_distance())
    if (mySonar.critical_Distance_Reached() and use_sonar) or (use_lidar and myLidar.critical_Distance_Reached()):
        return True
    return False

# def print_Logs(mySonar, myLidar):
#     """ Function that takes as input two ojects of type sonar and lidar
#     and prints their log, the history of the data they read """
#     plt.figure()
#     plt.title("Ranged mesured by the sonar and the lidar")
#     plt.xlabel("Time in s")
#     plt.ylabel("Distance in cm")
#     mySonar.see_log()
#     myLidar.see_log()
#     plt.legend()
#     plt.show()   

# -------- main ---------

# We start by initializing our 2 sensors.
# We can give as parameter the critical distance of the sensor (under which we detect an obstacle)
# By default, the critical distance is 50 cm
S = Sonar(50)
L = Lidar(30)
mesure_lidar = False
mesure_sonar = True
utilisation_filtre = True

runningTime = 60 # Time in s for which we collect the data
while time.time() - S.startTime < runningTime:
    if obstacle_Detected(S,L,mesure_lidar,mesure_sonar,debug = True, filtre = utilisation_filtre):

        if int(time.time() * 2) % 2 == 0:
            GPIO.output(buzzer, GPIO.HIGH)
        if int(time.time() * 2) % 2 == 1:
            GPIO.output(buzzer, GPIO.LOW)
        print("\\n TU VAS TE PRENDRE LE MUR !! \\n")

    else :
        GPIO.output(buzzer, GPIO.LOW)


GPIO.output(buzzer, GPIO.LOW)
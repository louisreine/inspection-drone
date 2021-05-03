"""
Date : 03/05/2020
Authors : Thibaud Cambronne and Louis Reine


In this file, we use 3 classes (1 parent and 2 childs) to read range data.
Two sensors (a sonar and a lidar) are connected to an Arduino
and the Arduino sends their data to the raspberry throught it's seria port (USB)
"""

import serial
import time
import matplotlib.pyplot as plt

ser=serial.Serial("/dev/ttyACM0",115200)  # change ACM number as found from ls /dev/tty/ACM*
ser.baudrate=115200


def readSensorsLine():
    """Function that reads the last line of data given by the sensors"""
    ligne_lue = str(ser.readline())
    # We are going to avoid some reading mistakes caused by an incomplete line
    if len(ligne_lue) not in [20,21,22]: # The lenght of a line can be 20, 21 or 22, depending on the value of the range
        ligne_lue = "No Data" # If the lenghts of the line read is not 20, 21 nor 22, there is an error
    return ligne_lue

# -------- Parent ---------

class RangeSensor:
    def __init__(self, _criticalDistance = 30):
        self.range = 0
        self.criticalDistance = _criticalDistance
        self.log = [] # type: List[int]
        # log stores range values for visualization
        self.time_log = [] # type: List[float]
        # time_log stores the time the range was stored in the log (in ms)
        self.startTime = time.time() # stores the time at which the object was created (in ms)
    
    def get_last_index_range(self, range_string, first_index_range):
        """ Function that takes as input a string to read and the index of the first byte of the data we want to get (in cm)
        and returns the index of the last byte containing the data in cm.
        This function is used with the data given in the following example format : b'Sonar_Range:57\\r\\n'
        where we want to recover the '57'"""
        
        first_index_range += 1
        while True:
            if range_string[first_index_range:first_index_range+1] == "\\":
                return first_index_range
            first_index_range += 1
            
    def critical_Distance_Reached(self):
        """ Checks if the range is inferior to the critical distance of the sensor
        Returns True if that's the case, False otherwise"""
        if self.range < self.criticalDistance:
            return True
        return False
        
    
    def get_distance(self):
        """Returns the last distance read by the sensor"""
        return self.range
    
    def set_distance(self, distanceValue):
        """Modifies the range value stored by the object.
        In the meantimes, it stores this value and the time it was written in a list"""
        self.range = distanceValue
        self.log.append(distanceValue)
        self.time_log.append(time.time() - self.startTime)
    
    def see_log(self):
        """Plots the history of the data"""
        plt.plot(self.time_log, self.log)

# -------- Childs ---------
    
class Sonar(RangeSensor):
    "Class for a specific range sensor : the MaxSonar I2CXL EZ0"
    def __init__(self, _criticalDistance = 30):
        "Constructor"
        super().__init__(_criticalDistance)
        
    def read_distance(self, read_ser = readSensorsLine()):
        """Gets the range given by the sensor connected to the Arduino
        Returns False if no data is read and True otherwise.
        
        Takes as input the line containing the data we want to read from the Arduino.
        If no line is given as input, we read a new line.
        """
        if read_ser[2] == 'S':
            # The line with the data looks like that : "b'Sonar_Range:57\\r\\n'"
            #Thus, a line starting by S contains data from the Sonar
            last_index_range = self.get_last_index_range(read_ser, 14)
            self.set_distance(int(read_ser[14:last_index_range]))
            return True
        return False
            
class Lidar(RangeSensor):
    "Class for a specific range sensor : the TFMini Plus"
    def __init__(self, _criticalDistance = 30):
        "Constructor"
        super().__init__(_criticalDistance)
    
    def read_distance(self, read_ser = readSensorsLine()):
        """Gets the range given by the sensor connected to the Arduino
        Returns False if no data is read and True otherwise.
        
        Takes as input the line containing the data we want to read from the Arduino.
        If no line is given as input, we read a new line.
        """
        if read_ser[2] == 'L':
            # The line with the data looks like that : "b'Lidar_Range:174\\r\\n'"
            # Thus, a line starting by S contains data from the Sonar
            last_index_range = self.get_last_index_range(read_ser, 14)
            self.set_distance(int(read_ser[14:last_index_range]))
            return True
        return False

def debug():
    for i in range(20):
        print(readSensorsLine())

# -------- main ---------

S = Sonar(40)
L = Lidar()

while time.time() - S.startTime < 15:
    read_serial = readSensorsLine()
    if S.read_distance(read_serial):
        print(f"Sonar range: {S.get_distance()}")
    if L.read_distance(read_serial):
        print(f"Lidar range: {L.get_distance()}")
    if S.critical_Distance_Reached() or L.critical_Distance_Reached():
        print("\\n TU VAS TE PRENDRE LE MUR !! \\n")

plt.figure()
S.see_log()
L.see_log()
plt.show()
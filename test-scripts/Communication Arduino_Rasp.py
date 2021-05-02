import serial
import time

ser=serial.Serial("/dev/ttyACM0",115200)  #change ACM number as found from ls /dev/tty/ACM*
ser.baudrate=115200

def get_last_index_range(range_string, first_index_range):
    # Mettre en fonction de la classe parent
    first_index_range += 2
    while True:
        if range_string[first_index_range:first_index_range+1] == "\\":
            return first_index_range
        first_index_range += 1

class Sonar:
    "Definition d'un capteur de distance : Le MaxSonar I2CXL EZ0"
    def __init__(self):
        "Constructeur"
        self.range = 0
        
    @classmethod
    def get_distance(self):
        """Permet de récupérer la distance donée par le capteur"""
        while True:
            read_ser=str(ser.readline())
            if read_ser[2] == 'S':
                # La ligne comportant la donnée sur le sonar est : "b'Sonar_Range:57\\r\\n'"
                # Ainsi, une ligne commençant par R contient la donnée du Sonar
                last_index_range = get_last_index_range(read_ser, 14)
                self.range = int(read_ser[14:last_index_range])
                return self.range
            
class Lidar:
    "Definition d'un capteur de distance : le TFMini Plus"
    def __init__(self):
        "Constructeur"
        self.range = 0
    
    @classmethod
    def get_distance(self):
        """Permet de récupérer la distance donée par le capteur"""
        while True:
            # On fait une boucle infinie mais on attend en fait très peu (au max on itère 2 fois)
            # puisque une ligne sur 2 lues est la bonne
            read_ser=str(ser.readline())
            if read_ser[2] == 'L':
                # La ligne comportant la donnée sur le lidar est : "b'Lidar_Range:174\\r\\n'"
                # Ainsi, une ligne commençant par L contient la donnée du Lidar
                last_index_range = get_last_index_range(read_ser, 14)
                self.range = int(read_ser[14:last_index_range])
                return self.range

S = Sonar()
print(S.get_distance())
L = Lidar()
print(L.get_distance())
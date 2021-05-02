# https://github.com/custom-build-robots/ultrasonic-sensor-MaxBotix-I2CXL-MaxSonar/blob/master/test-program.py
#coding:

# Connection sur les pins de gauche en partant de celui du haut :
# Connecter le fil rouge sur 3.3 V, le fil vert directement en dessous (SDA)
# Le fil bleu à nouveau juste dessous (SCL) et le fil noir sur la masse (en sauter un en dessous)

# Pour le maxbotix sonar, l'adresse I2C est 112 soit 0x70 en hexadecimal
# Pour vérifier, utiliser dans le terminal la commande : sudo i2cdetect -y 1

import smbus2 #https://pypi.org/project/smbus2/ pour la documentation
import time
import datetime
import matplotlib.pyplot as plt
import os # pour la fonction chdir qui permet de changer le repértoire de travail

address = 0x70
time_intervalle = 0.5

def write(bus, value):
    bus.write_byte(address, value)
    return -1

def distance(bus):    
    write(bus, 0x51)
    time.sleep(time_intervalle) # la doc conseille d'attendre 80ms entre l'envoi
    # de la commande "range_reading" et la lecture de la distance
    
    val = bus.read_word_data(address, 0xe1)
    # On fait range_high*256 + range_low
    dist = (val >> 8) & 0xff | (val & 0xff)
        
    return [dist,val,val>>8,(val & 0xff)]

def acquisition_stockage_distances(secondes):
    """ Fonction qui lance une acquistion de données du capteur de distances pendant
    le nombre de secondes donné en entrée """
    
    os.chdir("/home/pi/Desktop")
    date = datetime.datetime.today()
    file = open(f"Donnees_sonar_{date}.txt", 'w')
    t_init = time.time()
    Donnees = []
    Donnees_brutes = []
    Donnees_high = []
    elapsed_time = time.time() - t_init
    while elapsed_time < secondes:
        # create an object with class smbus2
        bus = smbus2.SMBus(1) # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
        acquisition = distance(bus)
        Donnees.append(acquisition[0])
        Donnees_brutes.append(acquisition[1])
        Donnees_high.append(acquisition[2])
        time.sleep(0.1)# il faut attendre 100ms entre chaque mesure (le temps que l'onde
        # acoustique se dissipe)
        file.write(f'{elapsed_time};{acquisition[0]}\n')
        elapsed_time = time.time() - t_init
    file.close()
    return Donnees, Donnees_brutes

def acquisition_affichage_distances(secondes):
    t_init = time.time()
    Donnees = []
    Donnees_brutes = []
    Donnees_low = []
    Donnees_high = []
    T = []
    elapsed_time = time.time() - t_init
    
    while elapsed_time < secondes:
        # create an object with class smbus2
        bus = smbus2.SMBus(1) # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1)
        acquisition = distance(bus)
        Donnees.append(acquisition[0])
        Donnees_brutes.append(acquisition[1])
        Donnees_high.append(acquisition[2])
        Donnees_low.append(acquisition[3])
        time.sleep(time_intervalle) # il faut attendre 100ms entre chaque mesure (le temps que l'onde
        # acoustique se dissipe)
        T.append(elapsed_time)
        elapsed_time = time.time() - t_init

    return Donnees, Donnees_brutes, Donnees_high, Donnees_low, T
    
    
if __name__ == '__main__':
    print("Launched the sonar code, have fun !")
    Donnees, Donnees_brutes, Donnees_high, Donnees_low, T = acquisition_affichage_distances(10)
    
    plt.plot(T, Donnees)
    plt.plot(T, Donnees_brutes)
    plt.plot(T, Donnees_high)
    plt.plot(T, Donnees_low)
    plt.legend(["donnes", "donnees brutes","high","low"])
    plt.show()
            


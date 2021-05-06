from InspectionDrone import InspectionDrone
import time

bobby = InspectionDrone('/dev/serial0', baudrate=115200, two_way_switches=[7, 8],
                        three_way_switches=[1, 2, 3, 4, 5, 6, 8, 9, 10, 11], buzzerPin=23)

while True:
    print(bobby)
    bobby.update_switch_states()
    print bobby.switches
    time.sleep(0.4)
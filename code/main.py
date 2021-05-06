from InspectionDrone import InspectionDrone, Switch
import time

# Let's recall the T12K mapping :
# SF(2):7                                               HS(2, push):8
#     SE(3):5                                       SG(3):6
#       SR(3):9 SB(3):10 LD(An): RD(An) SC(3):11 SD(3):12

bobby = InspectionDrone('/dev/serial0', baudrate=115200, two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12], buzzer_pin=23)

while True:
    # ---------------------------------------------------------------
    #                         Updating command
    # ---------------------------------------------------------------

    bobby.update_switch_states()

    if bobby.switches[5].is_up():
        print("working duuude")


    time.sleep(0.01)

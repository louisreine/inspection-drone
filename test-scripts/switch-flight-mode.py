from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import logging

print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)


def set_rc(chnum, v):
    inspection_drone._channels._update_channel(str(chnum), v)


# Just a function to be sure that all rc channels are written in the vehicle.channel object
@inspection_drone.on_message('RC_CHANNELS')
def RC_CHANNEL_listener(vehicle, name, message):
    set_rc(1, message.chan1_raw)
    set_rc(2, message.chan2_raw)
    set_rc(3, message.chan3_raw)
    set_rc(4, message.chan4_raw)
    set_rc(5, message.chan5_raw)
    set_rc(6, message.chan6_raw)
    set_rc(7, message.chan7_raw)
    set_rc(8, message.chan8_raw)
    set_rc(9, message.chan9_raw)
    set_rc(10, message.chan10_raw)
    set_rc(11, message.chan11_raw)
    set_rc(12, message.chan12_raw)
    set_rc(13, message.chan13_raw)
    set_rc(14, message.chan14_raw)
    set_rc(15, message.chan15_raw)
    set_rc(16, message.chan16_raw)
    vehicle.notify_attribute_listeners('channels', vehicle.channels)

    if message.chan8_raw > 1500:
        print("Launching code !")
        drone_change_mode(vehicle)


def drone_change_mode(drone):
    start_time = time.time()
    total_time = 20
    elapsed_time = time.time() - start_time
    while elapsed_time < total_time:
        if elapsed_time < total_time / 3:
            drone.mode = VehicleMode("AUTO")

        if total_time / 3 < elapsed_time < total_time * 2 / 3:
            drone.mode = VehicleMode("GUIDED")

        if total_time * 2 / 3 < elapsed_time < total_time:
            drone.mode = VehicleMode("AUTO")

        print(drone.mode)
        drone_change_mode()



def set_rc(chnum, v):
    inspection_drone._channels._update_channel(str(chnum), v)


while True:
    print("this is boring testing")
    time.sleep(0.1)

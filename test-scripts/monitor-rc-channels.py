from dronekit import connect
import time

print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)


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

def set_rc(chnum, v):
    inspection_drone._channels._update_channel(str(chnum), v)


while True:
    # Get all channel values from RC transmitter
    print "Channel values from RC Tx:", inspection_drone.channels
    time.sleep(0.5)

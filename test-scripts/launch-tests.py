from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import sys
import importlib
import os.path

test_scripts = []
for name_script in sys.argv[1:]:
    try:
        test_scripts.append(importlib.import_module(name_script))
    except ImportError:
        print("%s was not imported because it doesn't exist" % name_script)



# connection to drone, using serial

print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)

# Management of RC Channels to launch code
# Every Channel is attributed to a knob on the transmitter.
# We use a FUTABA TK12 as our main trasmitter, on which we mapped the different RC Channels
# See the mapping in the "mapping.txt"

def set_rc(chnum, v):
    inspection_drone._channels._update_channel(str(chnum), v)

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
        if 0 < message < 200 :
            test_scripts[0].launch_script(inspection_drone)
        if 200 < message < 500 :
            test_scripts[1].launch_script(inspection_drone)
        if 500 < message < 900 :
            test_scripts[2].launch_script(inspection_drone)


from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# connection to drone, using tcp

print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=921600)

def test_regain_control():
    # Printing the drone current flight mode.
    # If the drone is passed in GUIDED flight mode, it will print something else and
    # Try to see if you can regain control at every moment
    is_being_tested = True
    while True:
        print inspection_drone.mode
        if inspection_drone.mode == VehicleMode('GUIDED') and is_being_tested:
            print 'DRONE IS MOVING AUTONOMOUSLY'
        time.sleep(1)

test_regain_control()
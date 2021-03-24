from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# connection to drone, using tcp

print 'Connecting to drone'
inspection_drone = connect('tcp:127.0.0.1:5762', wait_ready=True)


# Function taken from the dronekit documentation
# Used here to test different movement and the
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = inspection_drone.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        inspection_drone.send_mavlink(msg)
        time.sleep(1)


def test_go_forward():
    # Printing the drone current flight mode.
    # If the drone is passed in GUIDED flight mode, it goes forward for 5s at 0.3m/s.
    # Try to see if you can regain control at every moment
    is_being_tested = True
    while True:
        print inspection_drone.mode
        if inspection_drone.mode == VehicleMode('GUIDED') and is_being_tested:
            print 'DRONE IS MOVING AUTONOMOUSLY'
            send_ned_velocity(-0.3, 0, 0, 5)
            is_being_tested = False
        time.sleep(1)


print 'launching test-going-forward'
test_go_forward()
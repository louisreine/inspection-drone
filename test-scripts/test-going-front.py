from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import logging

# connection to drone, using tcp
print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=921600)

# Log handling. Logs are saved in test-going-front.py


logging.basicConfig(format='%(asctime)s %(message)s')
logging.basicConfig(filename='test-going-front.log', level=logging.DEBUG)

logging.info('test-going-front started')


# Function taken from the dronekit documentation :
# https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
# Used here to test movement going forward
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

    print "SENDING MAVLINK SIGNAL : GOING BACKWARD"
    inspection_drone.send_mavlink(msg)


def test_go_forward(drone, speed, test_time):
    # Printing the drone current flight mode.
    # When the drone is passed in GUIDED flight mode, it executes the following instructions :
    # stay stationary during 4 seconds
    # goes forward seconds at 0.3m/s during test_time
    # stop and stay stationary during 4 seconds
    # Try to see if you can regain control at every moment
    current_flight_mode = drone.mode
    is_being_tested = True
    launch_test = False
    total_test_time = test_time + 4 * 2
    print_time = 0
    while True:
        current_time = time.time()

        # Check when we first go into guided mode
        if current_flight_mode != drone.mode:
            if drone.mode == VehicleMode('GUIDED'):
                launch_test = True
                test_launch_time = time.time()
                logging.warn("LAUNCH THE TEST")
                print "LAUNCHED"
            current_flight_mode = drone.mode

        if (time.time() - print_time) > 2:
            print drone.mode
            print_time = time.time()

        if drone.mode == VehicleMode('GUIDED') and launch_test and is_being_tested:
            time_elapsed = current_time - test_launch_time

            # Wait still for 4 seconds
            if time_elapsed < 4:
                logging.info("We wait for 4 seconds")
                send_ned_velocity(0, 0, 0)

            # Perform the action for test_time
            if 4 < time_elapsed < 4 + test_time:
                logging.info("We go forward for %s seconds" % test_time)
                send_ned_velocity(speed, 0, 0)

            # Wait still for 4 seconds after the action
            if 4 + test_time < time_elapsed < total_test_time:
                logging.info("We wait for 4 seconds after performing the orders")
                send_ned_velocity(0, 0, 0)

            # If the test has been done, reset all parameters.
            if time_elapsed > total_test_time:
                logging.warn("TEST DONE")
                launch_test = False
                is_being_tested = False


print 'launching test-going-forward'
test_go_forward(inspection_drone, 0.1, 3)

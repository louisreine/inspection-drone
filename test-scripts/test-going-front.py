from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import logging

# connection to drone, using serial
print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)

# Log handling. Logs are saved in test-going-front.py


logging.basicConfig(format='%(asctime)s %(message)s')
logging.basicConfig(filename='test-going-front.log', level=logging.DEBUG)

logging.info('test-going-front started')


# Function taken from the dronekit documentation :
# https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html
# Used here to test movement going forward
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
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

    if velocity_x > 0:
        print "SENDING MAVLINK SIGNAL : GOING BACKWARD"
        logging.info("SENDING MAVLINK SIGNAL : GOING BACKWARD")
    elif velocity_x == 0:
        print "SENDING MAVLINK SIGNAL : STATIONARY"
        logging.info("SENDING MAVLINK SIGNAL : STATIONARY")
    else:
        print "MESSAGE WAS SENDED, IDK WHICH ONE"
        logging.info("MESSAGE WAS SENT IDK WHICH ONE")
    inspection_drone.send_mavlink(msg)


def test_go_forward(drone, speed, test_time):
    # Printing the drone current flight mode.
    # When the drone is passed in GUIDED flight mode, it executes the following instructions :
    # stay stationary during 4 seconds
    # goes forward seconds at speed m/s during test_time
    # stop and stay stationary during 4 seconds
    # Try to see if you can regain control at every moment
    print "Current flight mode is : %s " % drone.mode
    is_being_tested = True
    total_test_time = test_time + 4 * 2

    def launch_test():

        start_time = time.time()
        time_elapsed = time.time() - start_time
        print start_time
        logging.warn("STARTING TEST")
        logging.info("  ")
        while drone.mode == VehicleMode('GUIDED') and time_elapsed < total_test_time:

            print time_elapsed
            time_elapsed = time.time() - start_time

            # Wait still for 4 seconds
            if time_elapsed < 4:
                print "wait phase"
                logging.info("We wait for 4 seconds")
                send_ned_velocity(0, 0, 0)

            # Perform the action for test_time
            if 4 < time_elapsed < 4 + test_time:
                print "move phase"
                logging.info("We go forward for 4 seconds")
                send_ned_velocity(speed, 0, 0)

            # Wait still for 4 seconds after the action
            if 4 + test_time < time_elapsed < total_test_time:
                print "wait phase 2"
                logging.info("We wait for 4 seconds after performing the orders")
                send_ned_velocity(0, 0, 0)

        # When the test has been done, reset all parameters.
        logging.warn("TEST DONE,")

    def mode_callback(self, attr_name, value):
        print "Vehicule mode was changed to %s " % value
        if value == VehicleMode('GUIDED'):
            print "Callback triggered, launching test !"
            launch_test()

    drone.add_attribute_listener('mode', mode_callback)

    time.sleep(2)

    print 'added callback'

    while True:
        time.sleep(0.01)


print 'launching test-going-forward'
test_go_forward(inspection_drone, 0.5, 3)

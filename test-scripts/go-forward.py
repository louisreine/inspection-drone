from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import logging


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
        print "MESSAGE WAS SENT, IDK WHICH ONE"
        logging.info("MESSAGE WAS SENT IDK WHICH ONE")
    inspection_drone.send_mavlink(msg)


def launch_test(drone, speed, test_time):
    total_test_time = 4 * 2 + test_time
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


if __name__ == '__main__':
    print 'launching test-going-forward'

    # connection to drone, using serial
    print 'Connecting to drone'
    inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)

    # Log handling. Logs are saved in go-forward.py

    logging.basicConfig(format='%(asctime)s %(message)s')
    logging.basicConfig(filename='go forward.log', level=logging.DEBUG)

    logging.info('launching go forward')

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
            launch_test(vehicle, 0.1, 4)
        
        while True:
            time.sleep(0.01)
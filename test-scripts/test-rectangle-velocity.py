from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import logging

logging.basicConfig(format='%(asctime)s %(message)s')
logging.basicConfig(filename='test_rectangle_velocity.log', level=logging.DEBUG)

logging.info('test_rectangle_velocity started')

# connection to drone, using tcp for testing, serial when not tested
logging.info('connecting to drone')
print 'Connecting to drone'
inspection_drone = connect('tcp:127.0.0.1:5762', wait_ready=True)


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


# A test_function that order the drone to follow a rectangle
# @param side_length is the length of the sides of the rectangle
# @param total_time is the time the drone will take to follow the rectangle

def follow_rectangle(side_length, total_time):
    speed = 4 * side_length / total_time

    # if speed is over 5km/h (ie 1.38 m/s), we set it to
    if speed > 1.4:
        speed = 1
        logging.warning("Set speed for rectangle following was too fast, reducing it to 1 m/s")

    side_time = total_time / 4
    logging.info("Drone will follow a rectangle of side length %s in %s seconds" % (side_length, total_time))
    logging.warning('DRONE IS MOVING AUTONOMOUSLY')
    send_ned_velocity(-speed, 0, 0, side_time)
    time.sleep(1)
    send_ned_velocity(0, -speed, 0, side_time)
    time.sleep(1)
    send_ned_velocity(speed, 0, 0, side_time)
    time.sleep(1)
    send_ned_velocity(0, speed, 0, side_time)
    time.sleep(1)
    logging.warning('AUTONOMOUS DRIVING DONE')


def test_follow_rectangle():
    # Printing the drone current flight mode.
    # If the drone is passed in GUIDED flight mode, it goes forward for 5s at 0.3m/s.
    # Try to see if you can regain control at every moment
    is_being_tested = True
    while inspection_drone.location.global_relative_frame.alt > 0.5:

        # print "Mode : %s " % inspection_drone.mode
        # print "Global Location: %s" % inspection_drone.location.global_frame
        # print "Global Location (relative altitude): %s" % inspection_drone.location.global_relative_frame
        # print "Local Location: %s" % inspection_drone.location.local_frame  # NED

        if inspection_drone.mode == VehicleMode(
                'GUIDED') and is_being_tested and inspection_drone.location.global_relative_frame.alt > 1.5:
            follow_rectangle(5, 20)
            is_being_tested = False
        time.sleep(1)


print 'launching %s test file' % __file__
test_follow_rectangle()

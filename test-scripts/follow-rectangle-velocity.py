from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import logging




def send_ned_velocity(drone, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = drone.message_factory.set_position_target_local_ned_encode(
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
        drone.send_mavlink(msg)
        time.sleep(1)


# A test_function that order the drone to follow a rectangle
# @param side_length is the length of the sides of the rectangle
# @param total_time is the time the drone will take to follow the rectangle

def launch_test(drone, side_length, total_time):
    speed = 4 * side_length / total_time

    # if speed is over 5km/h (ie 1.38 m/s), we set it to
    if speed > 1.4:
        speed = 1
        logging.warning("Set speed for rectangle following was too fast, reducing it to 1 m/s")

    side_time = total_time / 4
    logging.info("Drone will follow a rectangle of side length %s in %s seconds" % (side_length, total_time))
    logging.warning('DRONE IS MOVING AUTONOMOUSLY')
    send_ned_velocity(drone, -speed, 0, 0, side_time)
    time.sleep(1)
    send_ned_velocity(drone, 0, -speed, 0, side_time)
    time.sleep(1)
    send_ned_velocity(drone, speed, 0, 0, side_time)
    time.sleep(1)
    send_ned_velocity(drone, 0, speed, 0, side_time)
    time.sleep(1)
    logging.warning('AUTONOMOUS DRIVING DONE')


if __name__ == '__main__':
    logging.basicConfig(format='%(asctime)s %(message)s')
    logging.basicConfig(filename='follow-rectangle-velocity.log', level=logging.DEBUG)

    logging.info('follow-rectangle-velocity started')

    # connection to drone, using tcp for testing, serial when not tested
    logging.info('connecting to drone')
    print 'Connecting to drone'
    inspection_drone = connect('/dev/serial0', wait_ready=True)

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
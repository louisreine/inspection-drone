from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# connection to drone, using serial
print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)

# ------------------
# Parameters
# ------------------
inspection_drone.selectedTest = 0
inspection_drone.testNumber = -1
inspection_drone.startTime = time.time()
inspection_drone.obstacleDetected = False
inspection_drone.timeLastObstacleDetected = 0
print_time = time.time()

# Speed and time going forward for test 1 : going forward
speedTest1 = 0.1
timeTest1 = 4

# Speed and side_length for test 2 : following a square pattern
speedTest2 = 0.1
sideLength = 1
timeSide = sideLength / speedTest2
totalTime = 4 * sideLength / speedTest2

# Speed and side_length for test 3 : Auto mode then Guided and stop when there is an obstacle.
SpeedTest3 = 0.1
timeTest3 = 4


# Management of RC Channels to launch code
# Every Channel is attributed to a knob on the transmitter.
# We use a FUTABA TK12 as our main trasmitter, on which we mapped the different RC Channels
# See the mapping in the "mapping.txt"


def set_rc(vehicle, chnum, v):
    vehicle._channels._update_channel(str(chnum), v)


@inspection_drone.on_message('RC_CHANNELS')
def RC_CHANNEL_listener(vehicle, name, message):
    set_rc(vehicle, 1, message.chan1_raw)
    set_rc(vehicle, 2, message.chan2_raw)
    set_rc(vehicle, 3, message.chan3_raw)
    set_rc(vehicle, 4, message.chan4_raw)
    set_rc(vehicle, 5, message.chan5_raw)
    set_rc(vehicle, 6, message.chan6_raw)
    set_rc(vehicle, 7, message.chan7_raw)
    set_rc(vehicle, 8, message.chan8_raw)
    set_rc(vehicle, 9, message.chan9_raw)
    set_rc(vehicle, 10, message.chan10_raw)
    set_rc(vehicle, 11, message.chan11_raw)
    set_rc(vehicle, 12, message.chan12_raw)
    set_rc(vehicle, 13, message.chan13_raw)
    set_rc(vehicle, 14, message.chan14_raw)
    set_rc(vehicle, 15, message.chan15_raw)
    set_rc(vehicle, 16, message.chan16_raw)
    vehicle.notify_attribute_listeners('channels', vehicle.channels)

    if message.chan7_raw > 1500:
        print("Obstacle Detected !")
        inspection_drone.obstacleDetected = True
        inspection_drone.timeLastObstacleDetected = time.time()

    if 0 < message.chan6_raw < 1200:
        vehicle.selectedTest = 0
    if 1200 < message.chan6_raw < 1800:
        vehicle.selectedTest = 1
    if 1800 < message.chan6_raw < 2000:
        vehicle.selectedTest = 2

    if message.chan8_raw > 1500:
        if vehicle.testNumber != -1 and (time.time() - vehicle.startTime) > 1:
            print("Launching code !")
            vehicle.mode = VehicleMode("GUIDED")
            vehicle.startTime = time.time()
            if 0 < message.chan6_raw < 1200:
                vehicle.testNumber = 0
            if 1200 < message.chan6_raw < 1800:
                vehicle.testNumber = 1
            if 1800 < message.chan6_raw < 2000:
                vehicle.testNumber = 2
        else:
            vehicle.testNumber = -1
            print("Stopped Test using Transmitter input")

def send_ned_velocity(drone, velocity_x, velocity_y, velocity_z):
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

    drone.send_mavlink(msg)


def send_mavlink_go_forward(drone, velocity):
    print()
    send_ned_velocity(drone, velocity, 0, 0)


def send_mavlink_go_left(drone, velocity):
    send_ned_velocity(drone, 0, -velocity, 0)


def send_mavlink_go_right(drone, velocity):
    send_ned_velocity(drone, 0, velocity, 0)


def send_mavlink_go_backward(drone, velocity):
    send_ned_velocity(drone, -velocity, 0, 0)


def send_mavlink_stay_stationary(drone):
    send_ned_velocity(drone, 0, 0, 0)


def print_no_spam(start_print_time, string):
    if time.time() - start_print_time > 2:
        print string


def is_in_auto_mode(drone):
    return drone.mode == VehicleMode("AUTO")


def is_in_guided_mode(drone):
    return drone.mode == VehicleMode("GUIDED")


while True:

    if inspection_drone.testNumber == 0 and is_in_guided_mode(inspection_drone):
        elapsed_time = time.time() - inspection_drone.startTime
        print(elapsed_time)

        if 0 < elapsed_time < 4:

            send_mavlink_stay_stationary(inspection_drone)

        elif 4 < elapsed_time < 4 + timeTest1:

            send_mavlink_go_forward(inspection_drone, speedTest1)

        elif 4 + timeTest1 < elapsed_time < 8 + timeTest1:

            send_mavlink_stay_stationary(inspection_drone)

        elif 8 + timeTest1 < elapsed_time:

            inspection_drone.testNumber = -1
            inspection_drone.startTime = time.time()

    if inspection_drone.testNumber == 1 and is_in_guided_mode(inspection_drone):
        elapsed_time = time.time() - inspection_drone.startTime
        print(elapsed_time)
        if 0 < elapsed_time < timeSide:
            send_mavlink_go_forward(inspection_drone, speedTest2)

        if timeSide < elapsed_time < 2 * timeSide:
            send_mavlink_go_left(inspection_drone, speedTest2)

        if 2 * timeSide < elapsed_time < 3 * timeSide:
            send_mavlink_go_backward(inspection_drone, speedTest2)

        if 3 * timeSide < elapsed_time < 4 * timeSide:

            send_mavlink_go_forward(inspection_drone, speedTest2)

        elif 4 * timeSide < elapsed_time:

            inspection_drone.testNumber = -1
            inspection_drone.startTime = time.time()

    if inspection_drone.testNumber == 2:

        if is_in_guided_mode(inspection_drone) and not inspection_drone.obstacleDetected:
            if time.time() - inspection_drone.timeLastObstacleDetected > 2:
                inspection_drone.mode = VehicleMode("AUTO")
            else:
                send_mavlink_stay_stationary(inspection_drone)

        if is_in_guided_mode(inspection_drone) and inspection_drone.obstacleDetected:
            send_mavlink_stay_stationary(inspection_drone)

        if is_in_auto_mode(inspection_drone):
            if inspection_drone.obstacleDetected:
                inspection_drone.mode = VehicleMode("GUIDED")
                send_mavlink_stay_stationary(inspection_drone)

    print_no_spam(print_time, 'actual test is : %s ' % inspection_drone.selectedTest)

    if time.time() - print_time > 2:
        print_time = time.time()
    time.sleep(0.01)

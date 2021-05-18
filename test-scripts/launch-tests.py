from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import RPi.GPIO as GPIO
from RangeSensors import *

# Select GPIO mode
GPIO.setmode(GPIO.BCM)
# Set buzzer - pin 23 as output
buzzer = 23
GPIO.setup(buzzer, GPIO.OUT)

# connection to drone, using serial
print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)
print 'Successfully connected to drone !'
# ------------------
# Parameters
# ------------------

inspection_drone.selectedTest = 0
inspection_drone.testNumber = -1
inspection_drone.timeSinceLastLaunchInput = time.time()
inspection_drone.startTime = time.time()
inspection_drone.obstacleDetected = False
inspection_drone.timeLastObstacleDetected = 0
print_time = time.time()

# Speed and time going forward for test 1 : going forward
speedTest1 = 1
timeTest1 = 4

# Speed and side_length for test 2 : following a square pattern
speedTest2 = 1
sideLength = 5
timeSide = sideLength / speedTest2
totalTime = 4 * sideLength / speedTest2

# Speed and side_length for test 3 : Auto mode then Guided and stop when there is an obstacle.
use_sonar = False
use_lidar = True
sonar = Sonar(200)
lidar = Lidar(30)


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

    if 0 < message.chan6_raw < 1200:
        vehicle.selectedTest = 0
    if 1200 < message.chan6_raw < 1800:
        vehicle.selectedTest = 1
    if 1800 < message.chan6_raw < 2000:
        vehicle.selectedTest = 2

    if message.chan8_raw > 1500:
        if vehicle.testNumber == -1 and (time.time() - vehicle.startTime) > 0.4:
            print("Launching code !")
            vehicle.startTime = time.time()
            if 0 < message.chan6_raw < 1200:
                vehicle.testNumber = 0
                vehicle.mode = VehicleMode("GUIDED")
            if 1200 < message.chan6_raw < 1800:
                vehicle.testNumber = 1
                vehicle.mode = VehicleMode("GUIDED")
            if 1800 < message.chan6_raw < 2000:
                vehicle.testNumber = 2
        else:
            vehicle.testNumber = -1
            print("Stopped Test using Transmitter input")


def send_local_frame_offset_velocity(drone, velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = drone.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    drone.send_mavlink(msg)


def send_mavlink_go_forward(drone, velocity):
    print "Going forward"
    send_local_frame_offset_velocity(drone, velocity, 0, 0)


def send_mavlink_go_left(drone, velocity):
    print "Going left"
    send_local_frame_offset_velocity(drone, 0, -velocity, 0)


def send_mavlink_go_right(drone, velocity):
    print "Going right"
    send_local_frame_offset_velocity(drone, 0, velocity, 0)


def send_mavlink_go_backward(drone, velocity):
    print "Going backward"
    send_local_frame_offset_velocity(drone, -velocity, 0, 0)


def send_mavlink_stay_stationary(drone):
    print "Stopping"
    send_local_frame_offset_velocity(drone, 0, 0, 0)


def print_no_spam(start_print_time, string):
    if time.time() - start_print_time > 2:
        print string


def is_in_auto_mode(drone):
    return drone.mode == VehicleMode("AUTO")


def is_in_guided_mode(drone):
    return drone.mode == VehicleMode("GUIDED")


def obstacle_Detected(mySonar, myLidar, use_lidar, use_sonar, debug=False):
    """ Function that takes as input two objects of type sonar and lidar
    and that returns True if the distance they read is inferior to the critical distance defined for each of them
    The param debug has to be set to True if you want the data to be printed when uptdated"""
    read_serial = readSensorsLine()
    if mySonar.read_distance(read_serial) and debug:
        print "Sonar range:" + str(mySonar.get_distance())
    if myLidar.read_distance(read_serial) and debug:
        print "Lidar range:" + str(myLidar.get_distance())
    if (mySonar.critical_Distance_Reached() and use_sonar) or (use_lidar and myLidar.critical_Distance_Reached()):
        return True
    return False


while True:

    inspection_drone.obstacleDetected = obstacle_Detected(sonar, lidar, use_lidar, use_sonar)

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

            send_mavlink_go_right(inspection_drone, speedTest2)

        elif 4 * timeSide < elapsed_time:

            inspection_drone.testNumber = -1
            inspection_drone.startTime = time.time()

    if inspection_drone.testNumber == 2:

        elapsed_time = time.time() - inspection_drone.startTime
        print(elapsed_time)
        print(inspection_drone.mode)

        if is_in_guided_mode(inspection_drone):
            send_mavlink_stay_stationary(inspection_drone)

        if inspection_drone.obstacleDetected:
            if not is_in_guided_mode(inspection_drone):
                inspection_drone.mode = VehicleMode("GUIDED")
                inspection_drone.testNumber = -1

            if int(time.time() * 2) % 2 == 0:
                GPIO.output(buzzer, GPIO.HIGH)
            if int(time.time() * 2) % 2 == 1:
                GPIO.output(buzzer, GPIO.LOW)
        else:
            GPIO.output(buzzer, GPIO.LOW)

    print_no_spam(print_time, 'actual test is : %s ' % inspection_drone.selectedTest)

    if inspection_drone.testNumber != -1:
        print "Test %s is running" % inspection_drone.testNumber

    else:
        GPIO.output(buzzer, GPIO.LOW)

    if time.time() - print_time > 2:
        print_time = time.time()
    time.sleep(0.01)

from dronekit import connect, VehicleMode
import time
from RangeSensors import *
import RPi.GPIO as GPIO

# Select GPIO mode
GPIO.setmode(GPIO.BCM)
# Set buzzer - pin 23 as output
buzzer = 23
GPIO.setup(buzzer, GPIO.OUT)

print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)

# Define global parameters
inspection_drone.mission_running = False
inspection_drone.obstacleDetected = False
inspection_drone.timeLastObstacleDetected = time.time()
startTime = time.time()
elapsedTime = 0


def set_rc(chnum, v):
    inspection_drone._channels._update_channel(str(chnum), v)


# Update rc channels object and add a transmitter failsafe. If SF is not high, no code is running
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

    if message.chan7_raw > 1500:
        vehicle.mission_running = True

    if message.chan7_raw <= 1500:
        vehicle.mission_running = False



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
    send_ned_velocity(drone, velocity, 0, 0)


def send_mavlink_go_left(drone, velocity):
    send_ned_velocity(drone, 0, -velocity, 0)


def send_mavlink_go_right(drone, velocity):
    send_ned_velocity(drone, 0, velocity, 0)


def send_mavlink_go_backward(drone, velocity):
    send_ned_velocity(drone, -velocity, 0, 0)


def send_mavlink_stay_stationary(drone):
    send_ned_velocity(drone, 0, 0, 0)
    print("Stopping")


def is_in_auto_mode(drone):
    return drone.mode == VehicleMode("AUTO")


def is_in_guided_mode(drone):
    return drone.mode == VehicleMode("GUIDED")


def obstacle_detected(mySonar, myLidar, debug=False):
    """ Function that takes as input two objects of type sonar and lidar
    and that returns True if the distance they read is inferior to the critical distance defined for each of them
    The param debug has to be set to True if you want the data to be printed when uptdated"""
    read_serial = readSensorsLine()
    if mySonar.read_distance(read_serial) and debug:
        print("Sonar range: %s" % mySonar.get_distance())
    if myLidar.read_distance(read_serial) and debug:
        print("Lidar range: %s" % myLidar.get_distance())
    if mySonar.critical_Distance_Reached() or myLidar.critical_Distance_Reached():
        return True
    return False


# Define Sonar and Lidar Objects
sonar = Sonar()
lidar = Lidar(40)

starTime = time.time()
while True:
    # Update time
    elapsedTime = time.time() - startTime

    inspection_drone.obstacleDetected = obstacle_detected(sonar, lidar)

    if inspection_drone.mission_running:

        print inspection_drone.mode

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

        if inspection_drone.obstacleDetected:
            print "Obstacle detected"
            if int(elapsedTime * 8) % 2 == 0:
                GPIO.output(buzzer, GPIO.HIGH)
            if int(elapsedTime * 8) % 2 == 1:
                GPIO.output(buzzer, GPIO.LOW)
        else:
            print "No obstacle"
            if int(elapsedTime) % 2 == 0:
                GPIO.output(buzzer, GPIO.HIGH)
            if int(elapsedTime) % 2 == 1:
                GPIO.output(buzzer, GPIO.LOW)
    else:
        GPIO.output(buzzer, GPIO.LOW)

    time.sleep(0.001)

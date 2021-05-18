import warnings
from dronekit import connect, VehicleMode, Vehicle
from pymavlink import mavutil
import time
import RPi.GPIO as GPIO


class InspectionDrone(object):
    def __init__(self, connection_string, baudrate, two_way_switches, three_way_switches, buzzer_pin=None):

        # Initialize buzzer, making sure it is down
        if buzzer_pin is None:
            self._buzzerPin = 23
        else:
            self._buzzerPin = buzzer_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._buzzerPin, GPIO.OUT)
        GPIO.output(self._buzzerPin, GPIO.LOW)

        try:
            print "Trying to connect to the drone"
            self.vehicle = connect(connection_string, baudrate, wait_ready=True)
        except Exception as e:
            print str(e)
            raise ValueError("Unable to connect to done")
        print "Success"
        # Initialise switch states, Relies on T12K mapping
        self.switches = {}
        for key in two_way_switches:
            self.switches[key] = Switch(2)
        for key in three_way_switches:
            self.switches[key] = Switch(3)

        def set_rc(vehicle, chnum, value):
            vehicle._channels._update_channel(str(chnum), value)

        @self.vehicle.on_message('RC_CHANNELS')
        def RC_CHANNEL_listener(vehicle, name, message):
            # Write the correct values to the vehicle._channels
            # (without this, vehicle._channels is not correctly updated (only 8 channels)
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

        self._two_way_switches = two_way_switches
        self._three_way_switches = three_way_switches
        self._start_time = time.time()
        self._mission_start_time = 0
        self._obstacle_detected = False
        self._timeLast_obstacleDetected = None
        self._elapsed_time_connexion = time.time() - self._start_time
        self._elapsed_time_mission = 0
        self._mission_running = False

    # Will update the switch. We enumerate every value in the vehicle.channels dictionary, and set switch mode
    # according to the mapping
    def update_switch_states(self):
        for index, (key, value) in enumerate(self.vehicle.channels.items()):

            if int(key) in self._two_way_switches:
                if value < 1500:
                    self.switches[int(key)].set_state("down")
                if value > 1500:
                    self.switches[int(key)].set_state("up")

            if int(key) in self._three_way_switches:
                if value < 1200:
                    self.switches[int(key)].set_state("down")
                if 1200 < value < 1800:
                    self.switches[int(key)].set_state("middle")
                if 1800 < value:
                    self.switches[int(key)].set_state("up")

    def update_time(self):
        self._elapsed_time_connexion = time.time() - self._start_time
        if self._mission_running:
            self._elapsed_time_mission = time.time() - self._mission_start_time

    def print_switches_states(self):
        print(' ; '.join("{}: {}".format(k, v) for k, v in self.switches.items()))

    def _send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
        """
        Move vehicle in direction based on specified velocity vectors.
        Modified version of the dronekit one, only sends 1 mavlink message
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        self.vehicle.send_mavlink(msg)

    def send_mavlink_go_forward(self, velocity):
        print "Going forward"
        self._send_ned_velocity(velocity, 0, 0)

    def send_mavlink_go_left(self, velocity):
        print "Going left"
        self._send_ned_velocity(0, -velocity, 0)

    def send_mavlink_go_right(self, velocity):
        print "Going right"
        self._send_ned_velocity(0, velocity, 0)

    def send_mavlink_go_backward(self, velocity):
        print "Going backward"
        self.send_ned_velocity(-velocity, 0, 0)

    def send_mavlink_stay_stationary(self):
        print "Stopping"
        self.send_ned_velocity(0, 0, 0)

    def is_in_auto_mode(self):
        return self.mode == VehicleMode("AUTO")

    def is_in_guided_mode(self):
        return self.mode == VehicleMode("GUIDED")

    def is_mission_running(self):
        return self._mission_running

    def launch_mission(self):
        self._mission_start_time = time.time()
        self._mission_running = True

    def abort_mission(self):
        self._mission_running = False

class Switch(object):
    """
    A SwitchState class to handle switch detection on RC Transmitter.
    Does not rely on transmitter mapping.
    """

    def __init__(self, number_of_states=None, initial_state=None):
        self._time_last_updated = time.time()
        if number_of_states is None:
            self.number_of_states = 3
        else:
            self.number_of_states = number_of_states
        if initial_state is None:
            self.state = "down"
        else:
            self.set_state(initial_state)

    def __str__(self):
        return self.state

    def set_state(self, state):
        if not (state in ["up", "middle", "down"]):
            raise ValueError("set_state string argument must be either up, middle or down")

        if self.number_of_states == 3:
            if self.state != state:
                self._time_last_updated = time.time()
            self.state = state

        elif self.number_of_states == 2 and state != "middle":
            if self.state != state:
                self._time_last_updated = time.time()
            self.state = state

        else:
            raise ValueError("middle is not an acceptable value for a 2-way switch")

    def is_up(self):
        return self.state == "up"

    def is_down(self):
        return self.state == "down"

    def is_middle(self):
        if self.number_of_states == 2:
            warnings.warn("Warning : you tried to check whether a 2 state switch could be in the middle state")
        return self.state == "middle"

    def was_updated_since(self, amount_of_time):
        """
        Function that checks if a switch was updated since amount_of_time time.
        Used mainly to avoid making multiple unwanted inputs.
        :param amount_of_time: an amount of time in seconds. Use decimal for time under a second
        :return: True: was updated since amount_of time, False, was no updated since amount_of_time
        """
        return time.time() - self._time_last_updated > amount_of_time

from dronekit import connect, VehicleMode, Vehicle
from pymavlink import mavutil
import time
import RPi.GPIO as GPIO


class InspectionDrone(object):
    def __init__(self, connection_string, baudrate, two_way_switches, three_way_switches, buzzerPin=None):

        # Initialize buzzer, making sure it is down
        if buzzerPin is None:
            self._buzzerPin = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._buzzerPin, GPIO.out)
        GPIO.output(self._buzzerPin, GPIO.LOW)

        try:
            self.vehicle = connect(connection_string, baudrate, wait_ready=True)
        except Exception as e:
            print str(e)
            raise ValueError("Unable to connect to done")

        # Initialise switch states, Relies on T12K mapping
        self.switches = {}
        for key in two_way_switches:
            self.switches[key] = Switch(2)
        for key in two_way_switches:
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
        self._startTime = time.time()
        self._obstacleDetected = False
        self._timeLastObstacleDetected = None

    # Will update the switch
    def update_switch_states(self):
        for index, (key, value) in enumerate(self.vehicle.channels):
            if key in self._two_way_switches:
                if value < 1500:
                    self.switches[key].set_state("down")
                if value > 1500:
                    self.switches[key].set_state("up")
            if key in self._three_way_switches:
                if value < 1200:
                    self.switches[key].set_state("down")
                if 1200 < value < 1800:
                    self.switches[key].set_state("middle")
                if 1800 < value:
                    self.switches[key].set_state("up")


class Switch(object):
    """
    A SwitchState class to handle switch detection on RC Transmitter.
    Does not rely on transmitter mapping.
    """

    def __init__(self, number_of_states=None, initial_state=None):
        if number_of_states is None:
            self.number_of_states = 3
        else:
            self.number_of_states = number_of_states
        if initial_state is None:
            self.state = "down"
        else:
            self.set_state(initial_state)

    def __str__(self):
        print self.state

    def set_state(self, state):
        if not (state in ["up", "middle", "down"]):
            raise ValueError("SwitchState must be either up, middle or down")

        if self.number_of_states == 3:
            self.state = state

        if self.number_of_states == 2 and state != "middle":
            self.state = state
        else:
            raise ValueError("middle is not an acceptable value for a 2-way switch")

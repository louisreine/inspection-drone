from InspectionDrone import InspectionDrone, Switch
import time
import RPi.GPIO as GPIO

# Let's recall the T12K mapping :
# SF(2):7                                               HS(2, push):8
#     SE(3):5                                       SG(3):6
#       SR(3):9 SB(3):10 LD(An): RD(An) SC(3):11 SD(3):12

critical_distance_sonar = 200
critical_distance_lidar = 300

# This script is the final script used for the last demo of the drone inspection project.
# Demo 1 makes the drone free and makes it stop when there is an obstacle
# Demo 2 makes the drone go forward for 5 seconds and use a PID to stop the drone
# Demo 3 makes the drone follow a automatic gps plan and makes it stop when there is an obstacle

bobby = InspectionDrone('/dev/serial0',
                        baudrate=115200,
                        two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        buzzer_pin=23,
                        critical_distance_sonar=200,
                        critical_distance_lidar=300)

demo_case = 0
debug_time = time.time()

while True:
    # ---------------------------------------------------------------
    #                         Updating command
    # ---------------------------------------------------------------

    bobby.update_switch_states()

    # Managing launch mission switch. Added latency to avoid not being able to stop the mission
    if bobby.switches[8].is_up() and bobby.mission_running() and bobby.time_since_mission_launch() > 0.4:
        bobby.abort_mission()

    if bobby.switches[8].is_up() and not bobby.mission_running() and bobby.switches[8].was_updated_since(0.4):
        bobby.launch_mission()

    # Managing switch demo_case selection. If we switch demo_mode during a mission, we abort the mission.
    if bobby.switches[6].is_up():
        if demo_case != 2:
            bobby.abort_mission()
        demo_case = 2

    if bobby.switches[6].is_middle():
        if demo_case != 1:
            bobby.abort_mission()
        demo_case = 1

    if bobby.switches[6].is_down():
        if demo_case != 0:
            bobby.abort_mission()
        demo_case = 0

    # ---------------------------------------------------------------
    #                         Updating time
    # ---------------------------------------------------------------

    bobby.update_time()

    # ---------------------------------------------------------------
    #                         Updating detection obstacle
    # ---------------------------------------------------------------

    bobby.update_detection(use_lidar=True, use_sonar=False, debug=False)

    # ---------------------------------------------------------------
    #                         Main Program
    # ---------------------------------------------------------------

    if bobby.mission_running():
        if demo_case == 0:
            if bobby.obstacle_detected():
                print "Obstacle Detected"
                if not bobby.is_in_guided_mode():
                    last_flight_mode = bobby.vehicle.mode
                    bobby.set_guided_mode()
                bobby.buzz(4)
                bobby.send_mavlink_stay_stationary()

            if not bobby.obstacle_detected():
                if not bobby.is_in_guided_mode():
                    bobby.stop_buzz()

                if bobby.is_in_guided_mode():
                    if bobby.time_since_last_obstacle_detected() > 5:
                        bobby.set_flight_mode(last_flight_mode)
                    bobby.buzz(2)

    if not bobby.mission_running():
        bobby.stop_buzz()

    # if bobby.mission_running():
    #     print "mission running !"
    #     print bobby._elapsed_time_mission
    #
    # if not bobby.mission_running():
    #     print "mission not running !"

    if time.time() - debug_time > 1:
        print "Demo number %s" % demo_case
        print "Is mission running : %s" % bobby.mission_running()
        print "Obstacle detected : %s" % bobby.obstacle_detected()
        print "param : %s " % bobby.vehicle.parameters["FLTMODE2"]
        debug_time = time.time()

    time.sleep(0.01)

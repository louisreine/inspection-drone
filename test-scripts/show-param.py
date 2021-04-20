from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)

print "\nPrint all parameters (iterate `inspection_drone.parameters`):"
for key, value in inspection_drone.parameters.iteritems():
    print " Key:%s Value:%s" % (key, value)

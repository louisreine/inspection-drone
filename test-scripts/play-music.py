from dronekit import connect

# connection to drone, using serial
print 'Connecting to drone'
inspection_drone = connect('/dev/serial0', wait_ready=True, baud=115200)

tune_to_play = 'T110L16O4B>P16DP16F#P16EP16F#P16DP16<A>P16C#P16F#P16EP16F#P16C#P16<B>P16DP16F#P16EP16F#P16DP16<A>P16C#P16F'


msg = inspection_drone.message_factory.play_tune(0, tune_to_play)
inspection_drone.send_mavlink(msg)
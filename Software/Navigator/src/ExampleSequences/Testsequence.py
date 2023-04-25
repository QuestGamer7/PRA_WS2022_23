from src.Connector import SurfaceComputerToAutopilot as SC2AP
from src.Controller import Commands
import time
import ms5837
from timeit import default_timer
import pymavlink

# Playground
def Testsequence_GroundControlStation():
        sensor = ms5837.MS5837_30BA()  # Use default I2C bus (1)
        sensor.init()
        sensor.read(ms5837.OSR_256)
        print(sensor.depth())

        # create connection from surface computer to autopilot
        print("Connecting to autopilot")
        master_SC2AP, boot_time = SC2AP.create_master()

        # receive information
        print("Receive packet")
        msg = SC2AP.recv_match(master_SC2AP)
        print(f"Packet received: {msg}")

        # clean up (disarm)
        Commands.disarm(master_SC2AP)
        master_SC2AP.motors_disarmed_wait()

        # arm ardusub
        Commands.arm(master_SC2AP)
        master_SC2AP.motors_armed_wait()

        # request current parameter
        print("request current depth")
        Commands.request_depth(master_SC2AP)
        # print current parameter
        print("read current depth")
        msg = Commands.get_surface_depth(master_SC2AP)
        # set depth
        print("set depth")
        Commands.set_surface_depth(master_SC2AP, -1000) # like set target depth but in cm??? Checked it, it has no effect... Why should a external pressure sensor read the depth (when the vehicle is considered at the surface)
        # read ack
        print("read acknowledgment")
        msg = Commands.get_surface_depth(master_SC2AP)
        # request parameter value to confirm
        print("request current depth")
        Commands.request_depth(master_SC2AP)
        # print new parameter
        print("read current depth")
        msg = Commands.get_surface_depth(master_SC2AP)

        # Check if surface depth has any effect
        #while(1):
        #        pass

        # set depth hold mode
        print("set depth hold mode")
        Commands.change_flightmode(master_SC2AP, mode='ALT_HOLD')

        # Check if surface depth has any effect
        #while(1):
        #        pass

        target_depth_set = False
        while target_depth_set == False:
                # set target depth
                print("set target depth")
                target_depth = -20
                Commands.set_target_depth(target_depth, master_SC2AP, boot_time) # surface depth shows a different value??? has to be looped in order to have any effect, prefer set surface depth instead???
                # read ack
                print("request current depth")
                Commands.request_depth(master_SC2AP)
                # print current parameter
                print("read current depth")
                msg = Commands.get_surface_depth(master_SC2AP)
                print(msg)


                # request pressure
                master_SC2AP.mav.param_request_read_send(
                master_SC2AP.target_system, master_SC2AP.target_component,
                b'BARO1_GND_PRESS',
                -1
                )

                # Print received parameter value
                print("parameter value:")
                message = master_SC2AP.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
                print(message)
                print('name: %s\tvalue: %d' %
                      (message['param_id'], message['param_value']))

                # Convert to depth
                pressure = message['param_value']
                g = 9.80665
                p_fresh = 997.0474
                p_salt = 1023.6
                depth = -pressure/(g * p_fresh)
                print(f"calculated depth: {depth}") # pressure not simulated??? same value as the "pressure sensor"??? how to get the real depth, which is necessary to loop until the target depth it is reached, since target_depth does not set surface_depth and has to be looped.
                print(f"sensor depth: {sensor.depth()}")


                #if float(msg['param_value']) != float(target_depth):
                #        print(float(msg['param_value']))
                #        print(float(target_depth))
                #        target_depth_set = False

        if False:
                # (set target yaw from 0 to 500 degrees in steps of 10, one update per second)
                roll_angle = pitch_angle = 0
                for yaw_angle in range(0, 500, 10):
                        Commands.set_target_attitude(roll_angle, pitch_angle, yaw_angle, master_SC2AP, boot_time)
                        time.sleep(1)  # wait for a second

                # spin the other way with 3x larger steps
                for yaw_angle in range(500, 0, -30):
                        Commands.set_target_attitude(roll_angle, pitch_angle, yaw_angle, master_SC2AP, boot_time)
                        time.sleep(1)

        # clean up (disarm) at the end
        #master_SC2AP.arducopter_disarm()
        #master_SC2AP.motors_disarmed_wait()

# Cleaned up version of the playground
def Testsequence_GroundControlStation_cleaned():
        print("Connecting external pressure sensor")
        # Pressure sensor (not working or no external pressure sensor exists in the SITL, at least on no simulated I2C connection)
        sensor = ms5837.MS5837_30BA(bus=1)  # Use default I2C bus (1)
        sensor.init()
        sensor.read(ms5837.OSR_256)
        print(sensor.depth())
        time.sleep(1)
        print(f"\n")

        # create connection from surface computer to autopilot
        print("Connecting to autopilot")
        master_SC2AP, boot_time = SC2AP.create_master()
        print(f"\n")

        # receive information
        #print("Receive packet")
        #msg = SC2AP.recv_match(master_SC2AP)
        #print(f"Packet received: {msg}")
        #print(f"\n")

        # request scaled pressure
        print("Request scaled pressure")
        Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=143)
        msg = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE3")
        print(msg)
        # Convert to depth
        depth = Commands.convert_pressure_to_depth(msg['press_abs']/1000, watertype='salt')
        print(f"calculated depth: {depth}")

        Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=137)
        msg = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE2")
        print(msg)
        # Convert to depth
        depth = Commands.convert_pressure_to_depth(msg['press_abs']/1000, watertype='salt')
        print(f"calculated depth: {depth}")

        Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=29)
        msg = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE")
        print(msg)
        # Convert to depth
        depth = Commands.convert_pressure_to_depth(msg['press_abs']/1000, watertype='salt')
        print(f"calculated depth: {depth}")

        # request altitude
        #Commands.request_scaled_pressure(master_SC2AP, param1=141)
        #msg = SC2AP.recv_match(master_SC2AP, mavpackettype="ALTITUDE")
        #msg = master_SC2AP.recv_match().to_dict()
        #print(msg)

        #location = master_SC2AP.location()
        #print(location)

        # clean up (disarm)
        print("Inital state")
        Commands.disarm(master_SC2AP)
        master_SC2AP.motors_disarmed_wait()
        print(f"\n")


        print("!!! Arming. Stay clear !!!")
        time_start = default_timer()
        countdown = 5
        while (default_timer()-time_start<countdown):
                print(round(countdown - (default_timer()-time_start)))
                time.sleep(1)
        # arm ardusub
        Commands.arm(master_SC2AP)
        master_SC2AP.motors_armed_wait()
        print("!!! Armed !!!")
        print(f"\n")

        # request current parameter
        print("request surface depth")
        Commands.request_depth(master_SC2AP)
        # print current parameter
        print("read current surface depth")
        msg = Commands.get_surface_depth(master_SC2AP)
        # set depth
        print("set surface depth")
        surface_depth = -30
        Commands.set_surface_depth(master_SC2AP, surface_depth) # when at the surface, the bottom of the submarine is at -30cm (thats just a guess. maybe thats what the parameter is good for ...)
        # read ack
        print("read acknowledgment")
        msg = Commands.get_surface_depth(master_SC2AP)
        # request parameter value to confirm
        print("request current surface depth")
        Commands.request_depth(master_SC2AP)
        # print new parameter
        print("read current surface depth")
        msg = Commands.get_surface_depth(master_SC2AP)

        if(msg['param_value']==surface_depth):
                print("surface depth successfully set")
        else:
                print("error: surface depth not confirmed")
        print(f"\n")

        # set depth hold mode
        print("set depth hold mode")
        Commands.change_flightmode(master_SC2AP, mode='ALT_HOLD')

        # Since no way was found to measure the pressure in the SITL or to get the altitude (via the alt command in the mavlink console it works), a timer is used instead
        time_start = default_timer()
        countdown = 10

        while (default_timer()-time_start)<countdown:
                # set target depth
                target_depth = -10
                print(f"set target depth: {target_depth}")
                Commands.set_target_depth(target_depth, master_SC2AP, boot_time) # surface depth shows a different value??? has to be looped in order to have any effect, prefer set surface depth instead???

                # request scaled pressure
                print("Request scaled pressure")
                Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=143)
                msg = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE3")
                print(msg)
                # Convert to depth
                depth = Commands.convert_pressure_to_depth(msg['press_abs'] / 1000, watertype='salt')
                print(f"calculated depth: {depth}")

                Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=137)
                msg = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE2")
                print(msg)
                # Convert to depth
                depth = Commands.convert_pressure_to_depth(msg['press_abs'] / 1000, watertype='salt')
                print(f"calculated depth: {depth}")

                Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=29)
                msg = SC2AP.recv_match(master_SC2AP, mavpackettype="SCALED_PRESSURE")
                print(msg)
                # Convert to depth
                depth = Commands.convert_pressure_to_depth(msg['press_abs'] / 1000, watertype='salt')
                print(f"calculated depth: {depth}")

                Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=33)
                msg = SC2AP.recv_match(master_SC2AP, mavpackettype="GLOBAL_POSITION_INT") #could be the correct msg for altitude
                print(msg)

                Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=62)
                msg = SC2AP.recv_match(master_SC2AP, mavpackettype="NAV_CONTROLLER_OUTPUT")
                print(msg)

                #Commands.request_scaled_pressure(master_SC2AP, param1=63)
                #msg = SC2AP.recv_match(master_SC2AP, mavpackettype="GLOBAL_POSITION_INT_COV")
                #print(msg)

                Commands.standard_request_msg(master_SC2AP, mavlink_msg_id=74)
                msg = SC2AP.recv_match(master_SC2AP, mavpackettype="VFR_HUD") #thats even exactly what is displayed in QGC
                print(msg)

                #Commands.request_scaled_pressure(master_SC2AP, param1=141)
                #msg = SC2AP.recv_match(master_SC2AP, mavpackettype="ALTITUDE")  # not receiving
                #print(msg)

                #Commands.request_scaled_pressure(master_SC2AP, param1=230)
                #msg = SC2AP.recv_match(master_SC2AP, mavpackettype="ESTIMATOR_STATUS") # not receiving
                #print(msg)


                # Print received parameter value (does not ensure that the next message is actually the pressure...)
                #message = Commands.read_pressure(master=master_SC2AP, num_sensor=1) # thats just the paramter, not the actual measurement!!!
                #print(f"sensor depth: {sensor.depth()}")

                # print the time left for reaching the target depth, before starting to rotate
                print(round(countdown - (default_timer() - time_start)))
                time.sleep(1)

        print(f"\n")

        print("rotate")
        # (set target yaw from 0 to 500 degrees in steps of 10, one update per second)
        roll_angle = pitch_angle = 0
        for yaw_angle in range(0, 360, 10):
                Commands.set_target_attitude(roll_angle, pitch_angle, yaw_angle, master_SC2AP, boot_time)
                time.sleep(1)  # wait for a second

        # spin the other way with 3x larger steps (in steps of 10*(-3)=-30, "-" to spin the other way)
        for yaw_angle in range(360, 0, -30):
                Commands.set_target_attitude(roll_angle, pitch_angle, yaw_angle, master_SC2AP, boot_time)
                time.sleep(1)

        # clean up (disarm) at the end
        master_SC2AP.arducopter_disarm()
        master_SC2AP.motors_disarmed_wait()

def Testsequence(useGroundControlStation = True, useCompanionComputer = False):
    if useGroundControlStation:
        Testsequence_GroundControlStation()

def Testsequence_cleaned(useGroundControlStation=True, useCompanionComputer=False):
        if useGroundControlStation:
                Testsequence_GroundControlStation_cleaned()
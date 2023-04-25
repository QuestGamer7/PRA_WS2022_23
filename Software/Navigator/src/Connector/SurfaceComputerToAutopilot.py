"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

from timeit import default_timer
# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
#  If using a companion computer
#  the default connection is available
#  at ip 192.168.2.1 and the port 14550
# Note: The connection is done with 'udpin' and not 'udpout'.
#  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
#  uses a 'udpbcast' (client) and not 'udpin' (server).
#  If you want to use QGroundControl in parallel with your python script,
#  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
#  E.g: --out udpbcast:192.168.2.255:yourport
def create_master(timeout=5):
    master = mavutil.mavlink_connection('udpin:127.0.0.1:14551') # If you’re running the GCS on the same machine as SITL then an appropriate output may already exist. Check this by calling output on the MAVProxy command prompt.
    # If the GCS is connected to the same port as the python script of the surface computer, it will not be able to listen and therefore display the movement, while the script is communicationg with the simulated autopilot!!!
    # Additional ports can be added by using: e.g. output add 192.168.14.82:14550
    # Thats how you can use the MAVProxy (UDP) forwarding to connect additional GCS etc.
    boot_time = time.time()
    # Make sure the connection is valid
    master.wait_heartbeat(timeout=timeout)

    return master, boot_time

def recv_match(master, timeout=5, mavpackettype = 'ATTITUDE'):
    # Get some information !
    time_start = default_timer()
    msg = None
    while (default_timer() - time_start)<timeout:
        print("waiting")
        try:
            msg = master.recv_match(type=mavpackettype).to_dict()
            break
        except:
            pass
        time.sleep(0.1)

    return msg

def get_depth(master, timeout=5):
    # Get some information !
    time_start = default_timer()
    msg = None
    while (default_timer() - time_start)<timeout:
        print("waiting")
        try:
            msg = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
            break
        except:
            pass
        time.sleep(0.1)

    return msg


# Output:
# {'mavpackettype': 'AHRS2', 'roll': -0.11364290863275528, 'pitch': -0.02841472253203392, 'yaw': 2.0993032455444336, 'altitude': 0.0, 'lat': 0, 'lng': 0}
# {'mavpackettype': 'AHRS3', 'roll': 0.025831475853919983, 'pitch': 0.006112074479460716, 'yaw': 2.1514968872070312, 'altitude': 0.0, 'lat': 0, 'lng': 0, 'v1': 0.0, 'v2': 0.0, 'v3': 0.0, 'v4': 0.0}
# {'mavpackettype': 'VFR_HUD', 'airspeed': 0.0, 'groundspeed': 0.0, 'heading': 123, 'throttle': 0, 'alt': 3.129999876022339, 'climb': 3.2699999809265137}
# {'mavpackettype': 'AHRS', 'omegaIx': 0.0014122836291790009, 'omegaIy': -0.022567369043827057, 'omegaIz': 0.02394154854118824, 'accel_weight': 0.0, 'renorm_val': 0.0, 'error_rp': 0.08894175291061401, 'error_yaw': 0.0990816056728363}
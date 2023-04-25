from Connector import CompanionComputerToAutopilot as CC2A
from Connector import CompanionComputerToQGroundControl as CC2QGC
from Connector import SurfaceComputerToAutopilot as SC2AP
from Controller import Commands
import time
import pymavlink.mavutil as mavutil
import ExampleSequences.Testsequence as examples

def main():
    ## create connection from companion computer to the autopilot
    #print("Connecting to autopilot")
    #master_CC2A = CC2A.create_master()
    #print("Connected")
    ## ping autopilot
    #print("Ping autopilot")
    #CC2A.wait_conn(master_CC2A)
    #print("Pinged")


    # create connection from companion computer to QGroundControl on the surface computer
    ##print("Connecting to surface computer")
    ##master_CC2QGC = CC2QGC.create_master()
    ##print("Connected")
    # send a text
    ##print("Sending text")
    ##CC2QGC.send_text(master_CC2QGC, "Hello QGroundControl")
    ##print("sent")

    # create connection from surface computer to autopilot
    #examples.Testsequence(useGroundControlStation=True)
    examples.Testsequence_cleaned(useGroundControlStation=True)



if __name__ == '__main__':
    main()






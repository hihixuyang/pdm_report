import sys
sys.path.append("./PidLa")
sys.path.append("./cortex")
sys.path.append("./crazy_lib")
import logging
import time
import struct
import pdb
import numpy as np
import math
import threading
import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie
from CortexDecoder import CortexDecoder
from PidLa_v3 import PidLa
from comData import ComData
from logData import LogData

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)
data_stabilizer = (0,0,0)

#Constant definition
LIM_XY = 1500.                          #Limits of Cortex zone
MAX_Z = 1500.                           #
MIN_Z = 0                               #
HOVER = 40000.0                         #Hover commad (approx)
MAX_THRUST = 60000                      #Limit on thrust command
MIN_THRUST = 30000                      #
ROLL_PITCH_LIM = 15.                    #Limit of Roll, Pitch command
YAWR_LIM = 90.                          #Limit of Yaw rate command
T_LOOP = 0.004                          #245 fps for Cortex
T_CORTEX_LOOP_FULL = 0.0002             #Time to read an image from the Cortex buffer when it's not empty
XYZ_TOL = 200                           #Limit for gain scheduling

class Logging:
    #
    #Logging class that logs the Stabilizer from a supplied
    #link uri.
    #
    def __init__(self, link_uri):
        #Initialize and run the example with the specified link_uri

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()
        
        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        
        print "Connecting to %s" % link_uri
        
        # Try to connect to the Crazyflie
        self._cf.open_link(link_uri)
        
        # Variable used to keep main loop occupied until disconnect
        self.is_connected = True
    
    def _connected(self, link_uri):
        #This callback is called form the Crazyflie API when a Crazyflie
        #has been connected and the TOCs have been downloaded.
        print "Connected to %s" % link_uri
        
        # The definition of the logconfig can be made before connecting
        
        self._lg_stab = LogConfig(name="Stabilizer", period_in_ms=10)
        self._lg_stab.add_variable("stabilizer.roll", "float")
        self._lg_stab.add_variable("stabilizer.pitch", "float")
        self._lg_stab.add_variable("stabilizer.yaw", "float")
        self._lg_stab.add_variable("acc.x", "float")
        
        self._cf.log.add_config(self._lg_stab)
        if self._lg_stab.valid:
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        else:
            print("Could not add logconfig since stab variables are not in TOC")

    def _stab_log_error(self, logconf, msg):
        #Callback from the log API when an error occurs
        print "Error when logging %s: %s" % (logconf.name, msg)

    def _stab_log_data(self, timestamp, data, logconf):
        global data_stabilizer
        data_stabilizer = (data["stabilizer.roll"], data["stabilizer.pitch"],data["stabilizer.yaw"])
       
    def _connection_failed(self, link_uri, msg):
        #Callback when connection initial connection fails (i.e no Crazyflie
        #at the speficied address)
        print "Connection to %s failed: %s" % (link_uri, msg)
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        #Callback when disconnected after a connection has been made (i.e
        #Crazyflie moves out of range)
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        #Callback when the Crazyflie is disconnected (called in all cases)"""
        print "Disconnected from %s" % link_uri
        self.is_connected = False
        
if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)    
    
    # Scan for Crazyflies and use the first one found
    ready = False
    while not ready:
        print "Scanning interfaces for Crazyflies..."
        available = cflib.crtp.scan_interfaces()
        print "Crazyflies found:"
        for i in available:
            print i[0]
        if len(available) > 0:
            CFuriLink=i[0]
            lin = Logging(CFuriLink)
            ready = True
        else:
            print "No Crazyflies found, cannot run example"
            time.sleep(2)
    #Let time to establish connection
    time.sleep(5)

    #Open the log file and the command file
    com = ComData('/Users/student/Desktop/loic_dubois')
    log = LogData('/Users/student/Desktop/loic_dubois/log')

    #Initialization and test of command
    xc = 0
    yc = 0
    zc = 0
    com.readData()
    xcNext = com.x
    ycNext = com.y
    zcNext = com.z
    if xcNext>LIM_XY:
        xcNext=LIM_XY
    elif xcNext<(-LIM_XY):
        xcNext=-LIM_XY
    if ycNext>LIM_XY:
        ycNext=LIM_XY
    elif ycNext<(-LIM_XY):
        ycNext=-LIM_XY
    if zcNext>MAX_Z:
        zcNext=MAX_Z
    elif zcNext<MIN_Z:
        zcNext=MIN_Z
    tComNext = com.t
    #Initialization of last known position
    xPre=0
    yPre=0
    zPre=0
    yawPre=0
    #PID Initialization and declaration (coarse and fine gain, anti-windup, saturation)
    kXY = .04
    kXYf = .02
    dXY = .6
    dXYf = 1.2
    pidX=PidLa(kXY,.2,dXY) #ZN: K=0.02, T=3.1
    pidX.set_saturation(-ROLL_PITCH_LIM, ROLL_PITCH_LIM)
    pidX.set_awindup('clamp')
    XisFine = False
    pidY=PidLa(kXY,.2,dXY) #idem
    pidY.set_saturation(-ROLL_PITCH_LIM, ROLL_PITCH_LIM)
    pidY.set_awindup('clamp')
    YisFine = False
    kZ = 6
    kZf = 4
    dZ = .8
    dZf = 2
    pidZ=PidLa(kZ,.8,dZ) #ZN: Kc=20, Tc = 3.045
    pidZ.set_saturation(MIN_THRUST-HOVER, MAX_THRUST-HOVER)
    pidZ.set_awindup('clamp')
    ZisFine = False
    pidYaw=PidLa(10,.5,0)
    pidYaw.set_saturation(-YAWR_LIM, YAWR_LIM)
    pidYaw.set_awindup('clamp')

    #Initialization of the loop
    quitCond = False
    count = 0
    ComStart = .5   #Time for takeoff (no command on roll, pitch and yaw rate)
    ComEnd = 35

    #Takeoff command
    lin._cf.commander.send_setpoint(0,0,0,HOVER-1000)

    #Cortex Connection
    MCAST_GRP = '225.1.1.1'
    MCAST_PORT = 2002
    deCortex = CortexDecoder(MCAST_GRP, MCAST_PORT)

    tStart=time.time()
    while not quitCond:
        tLoopStart =time.time()-tStart
        try:
            while True:
                #Read from Cortex until the buffer is empty -> Get last image
                t = time.time()
                deCortex.readFromSocket()
                dt = time.time()-t
                if dt>T_CORTEX_LOOP_FULL: break
            try:
                #get position of center of mass of the marker(s) (measure)
                deCortex.objs[0].computeCenter()
                xm=deCortex.objs[0].xC
                ym=deCortex.objs[0].yC
                zm=deCortex.objs[0].zC
            except:
                #if no marker visible
                print 'ERROR: No Object'
        
            #Test on measure (if out of Cortex zone)
            if xm>LIM_XY or xm<(-LIM_XY):
                xm=xPre
            if ym>LIM_XY or ym<(-LIM_XY):
                ym=yPre
            if zm>MAX_Z or zm<MIN_Z:
                zm=zPre
            #Measure vector
            mVect=([xm],[ym],[zm])

            #Save last value
            xPre=xm
            yPre=ym
            zPre=zm
        
            #Update and test command
            if tLoopStart > tComNext:
                xc = xcNext
                yc = ycNext
                zc = zcNext
                com.readData()
                if not com.end:
                    xcNext = com.x
                    ycNext = com.y
                    zcNext = com.z
                    if xcNext>LIM_XY:
                        xcNext=LIM_XY
                    elif xcNext<(-LIM_XY):
                        xcNext=-LIM_XY
                    if ycNext>LIM_XY:
                        ycNext=LIM_XY
                    elif ycNext<(-LIM_XY):
                        ycNext=-LIM_XY
                    if zcNext>LIM_XY:
                        zcNext=MAX_Z
                    elif zcNext<(-LIM_XY):
                        zcNext=MIN_Z
                    tComNext = com.t
                else:
                    #No more command
                    tComNext = 9999
    
            #Command vetor
            cVect = ([xc],[yc],[zc])

            #Compute rotation matrix from world to body frame and compute transform (cos, sin in radian)
            rom = math.radians(data_stabilizer[0])
            pim = -math.radians(data_stabilizer[1]) #data_stabiliser is inverted for pitch
            yam = math.radians(data_stabilizer[2])

            R_y = np.matrix([[math.cos(yam), math.sin(yam), 0],[-math.sin(yam), math.cos(yam), 0],[ 0, 0, 1]])
            R_p = np.matrix([[math.cos(pim), 0, -math.sin(pim)],[0, 1, 0],[math.sin(pim), 0, math.cos(pim)]])
            R_r = np.matrix([[1, 0, 0],[0, math.cos(rom), math.sin(rom)],[0, -math.sin(rom), math.cos(rom)]])
            R = R_r*R_p*R_y
        
            #Vectors in the body frame
            cBody = R*cVect
            mBody = R*mVect
        
            #PID Update
            #No update during takeoff
            if tLoopStart<ComStart or tLoopStart>ComEnd:
                uThrust = 0
                uRoll = 0
                uPitch = 0
                uYaw = 0
            else:
                #Switch between fine or coarse controller, reset only if change
                if zc-zm < XYZ_TOL and zc-zm > -XYZ_TOL:
                    if not ZisFine:
                        pidZ.Kp = kZf
                        pidZ.Td = dZf
                        pidZ.reset_I()
                        ZisFine = True
                else:
                    if  ZisFine:
                        pidZ.Kp = kZ
                        pidZ.Td = dZ
                        pidZ.reset_I()
                        ZisFine = False
                
                if xc-xm < XYZ_TOL and xc-xm > -XYZ_TOL:
                    if not XisFine:
                        pidX.Kp = kXYf
                        pidX.Td = dXYf
                        pidX.reset_I()
                        XisFine = True
                else:
                    if  XisFine:
                        pidX.Kp = kXY
                        pidX.Td = dXY
                        pidX.reset_I()
                        XisFine = False

                if yc-ym < XYZ_TOL and yc-ym > -XYZ_TOL:
                    if not YisFine:
                        pidY.Kp = kXYf
                        pidY.Td = dXYf
                        pidY.reset_I()
                        YisFine = True
                else:
                    if  YisFine:
                        pidY.Kp = kXY
                        pidY.Td = dXY
                        pidY.reset_I()
                        YisFine = False

                #Compute output
                uThrust = pidZ.get_u(mBody.item(2), cBody.item(2)) + HOVER #add a priori command
                uRoll = pidY.get_u(mBody.item(1), cBody.item(1))
                uPitch = pidX.get_u(mBody.item(0), cBody.item(0))
                uYaw = pidYaw.get_u(yam, 0)
            
            #Send Command only after ComStart until ComEnd
            if tLoopStart > ComEnd+.1:
                quitCond = True
            elif tLoopStart > ComEnd-T_LOOP and tLoopStart < ComEnd+T_LOOP:
                #Landing command
                comThread = threading.Thread(lin._cf.commander.send_setpoint(0,0,0,HOVER-3000))
                comThread.start()
            elif tLoopStart > ComStart and tLoopStart < ComEnd:
                lin._cf.commander.send_setpoint(-uRoll,uPitch,-uYaw,uThrust)
            elif tLoopStart < ComStart:
                pidZ.reset_I()
                pidY.reset_I()
                pidX.reset_I()
                pidYaw.reset_I()    

            #Log of data. Uncomment what you want to log
            log.newData('Count: ', count, time.time()-tStart) #Never comment Time for timeseries in matlab
            log.newData('Z: ', zc, zm)
            log.newData('Y: ', yc, ym)
            log.newData('X: ', xc, xm)
            log.newData('Thrust: ', (zc-zm)/zc, uThrust/HOVER)
            log.newData('Roll: ', (yc-ym)/LIM_XY, uRoll/ROLL_PITCH_LIM)
            log.newData('Pitch: ', (xc-xm)/LIM_XY, uPitch/ROLL_PITCH_LIM)
            log.newData('Yaw: ', -yam, uYaw/YAWR_LIM)

            count = count+1
                
        except:
            print 'ERROR: Could not read Cortex'

    time.sleep(.1)
    #Close files of command and log
    log.closeFile()
    com.closeFile()
    #Close link to CF
    closingThread = threading.Thread(target=lin._cf.close_link)
    closingThread.start()
    print("Link closed")

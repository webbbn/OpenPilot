import logging
import serial
import traceback

from openpilot.uavtalk.uavobject import *
from openpilot.uavtalk.uavtalk import *
from openpilot.uavtalk.objectManager import *
from openpilot.uavtalk.connectionManager import *
    
    
def _hex02(value):
    return "%02X" % value
    
class UavtalkDemo():
    
    def __init__(self):
        self.nbUpdates = 0
        self.lastRateCalc = time.clock()
        self.updateRate = 0
        self.objMan = None
        self.connMan = None
        
    def setup(self, port, uavdefPath):
        print "Opening Port"
        serPort = serial.Serial(port, 57600, timeout=.5)
        if not serPort.isOpen():
            raise IOError("Failed to open serial port")
        
        print "Creating UavTalk"
        self.uavTalk = UavTalk(serPort)
        
        print "Starting ObjectManager"
        self.objMan = ObjManager(self.uavTalk, uavdefPath)
        
        print "Starting UavTalk"
        self.uavTalk.start()
        
        print "Starting ConnectionManager"
        self.connMan = ConnectionManager(self.uavTalk, self.objMan)
        
        print "Connecting...",
        self.connMan.connect()
        print "Connected"
        
        print "Getting all Data"
        self.objMan.requestAllObjUpdate()
        
        print "SN:",
        sn = self.objMan.FirmwareIAPObj.CPUSerial.value
        print "".join(map(_hex02, sn))
        
    def stop(self):
        if self.uavTalk:
            print "Stopping UavTalk"
            self.uavTalk.stop()
        
    def showAttitudeViaObserver(self):
        print "Request fast periodic updates for AttitudeActual"
        self.objMan.AttitudeActual.metadata.telemetryUpdateMode.value = UAVMetaDataObject.UpdateMode.PERIODIC
        self.objMan.AttitudeActual.metadata.telemetryUpdatePeriod.value = 50
        self.objMan.AttitudeActual.metadata.updated()
        
        print "Install Observer for AttitudeActual updates\n"
        self.objMan.regObjectObserver(self.objMan.AttitudeActual, self, "_onAttitudeUpdate")
        # Spin until we get interrupted
        while True:
            time.sleep(1)
        
    def showAttitudeViaWait(self):
        print "Request fast periodic updates for AttitudeActual"
        self.objMan.AttitudeActual.metadata.telemetryUpdateMode.value = UAVMetaDataObject.UpdateMode.PERIODIC
        self.objMan.AttitudeActual.metadata.telemetryUpdatePeriod.value = 50
        self.objMan.AttitudeActual.metadata.updated()
        
        while True:
            self.objMan.AttitudeActual.waitUpdate()
            self._onAttitudeUpdate(self.objMan.AttitudeActual)
                    
    def showAttitudeViaGet(self):
        while True:
            self.objMan.AttitudeActual.getUpdate()
            self._onAttitudeUpdate(self.objMan.AttitudeActual)
        
    def _onAttitudeUpdate(self, args):
        self.nbUpdates += 1
        
        now = time.clock()    
        if now-self.lastRateCalc > 1:
            self.updateRate = self.nbUpdates/(now-self.lastRateCalc) 
            self.lastRateCalc = now
            self.nbUpdates = 0
            
        if self.nbUpdates & 1: 
            dot = "." 
        else: 
            dot= " "
        
        print " %s Rate: %02.1f Hz  " % (dot, self.updateRate),
            
        roll = self.objMan.AttitudeActual.Roll.value
        print "Roll: %-4d " % roll,
        i = roll/90
        if i<-1: i=-1
        if i>1: i= 1
        i = int((i+1)*15)
        print "-"*i+"*"+"-"*(30-i)+" \r",
        

if __name__ == '__main__':
    
    # Log everything, and send it to stderr.
    logging.basicConfig(level=logging.INFO)

    try:
        demo = UavtalkDemo()
        demo.setup(30-1, "D:\\Projects\\Fred\\OpenPilot\\git\\build\\uavobject-synthetics\\python")
        
        demo.showAttitudeViaObserver()   # will not return
            
    except KeyboardInterrupt:
        pass
    except Exception,e:
        print
        print "An error occured: ", e
        print
        traceback.print_exc()
    
    print
    
    try:
        demo.stop()
    except Exeption:
        pass
    
    raw_input("Press ENTER, the application will close")



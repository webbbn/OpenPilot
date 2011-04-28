import logging
import serial
import traceback

from openpilot.uavtalk.uavobject import *
from openpilot.uavtalk.uavtalk import *
from openpilot.uavtalk.objectManager import *
from openpilot.uavtalk.connectionManager import *


    
class UavtalkDemo():
    
    #UAVOBJDEF_PATH = 'D:\Projects\Fred\OpenPilot\SVN\ground\uavobjgenerator\debug\python'
    UAVOBJDEF_PATH = "D:\\Projects\\Fred\\OpenPilot\\git\\build\\uavobject-synthetics\\python"
    PORT = (30-1)
    
    def __init__(self):
        try:
            self.nbUpdates = 0
            
            print "Opening Port"
            serPort = serial.Serial(UavtalkDemo.PORT, 57600, timeout=.5)
            if not serPort.isOpen():
                raise IOError("Failed to open serial port")
            
            print "Creating UavTalk"
            self.uavTalk = UavTalk(serPort)
            
            print "Starting ObjectManager"
            self.objMan = ObjManager(self.uavTalk, UavtalkDemo.UAVOBJDEF_PATH)
            import attitudeactual
            
            print "Starting UavTalk"
            self.uavTalk.start()
            
            print "Starting ConnectionManager"
            self.connMan = ConnectionManager(self.uavTalk, self.objMan)
            
            print "Connecting...",
            self.connMan.connect()
            print "Connected"
            
            print "Getting all MetaData"
            self.objMan.requestAllMetaDataUpdate()
            
            print "Getting FirmwareIAP"
            self.objMan.waitObjUpdate(self.objMan.FirmwareIAPObj, request=True, timeout=1)
            print self.objMan.FirmwareIAPObj.CPUSerial.value
            
            print "Request fast periodic updates for AttitudeActual"
            self.objMan.AttitudeActual.metadata.telemetryUpdateMode.value = UAVMetaDataObject.UpdateMode.PERIODIC
            self.objMan.AttitudeActual.metadata.telemetryUpdatePeriod.value = 50
            self.uavTalk.sendObject(self.objMan.AttitudeActual.metadata)
            
            print "Install Observer for AttitudeActual updates\n"
            self.objMan.regObjectObserver(self.objMan.AttitudeActual, self, "_onAttitudeUpdate")
            
            
            # Spin until we get interrupted
            while True:
                time.sleep(1)
                
            print
                
        except KeyboardInterrupt:
            pass
        except Exception,e:
            print
            print "An error occured: ", e
            print
            traceback.print_exc()
        
        print "Stopping UavTalk"
        self.uavTalk.stop()
        raw_input("Press ENTER, the application will close")
        
    def _onAttitudeUpdate(self, args):
        attitudeObj = args[0]
        
        print "."*self.nbUpdates+" "*(10-self.nbUpdates),
        self.nbUpdates += 1
        if self.nbUpdates > 10:
            self.nbUpdates = 0
            
        print "Roll: %-4d " % attitudeObj.Roll.value,
        i = attitudeObj.Roll.value/90
        if i<-1: i=-1
        if i>1: i= 1
        i = int((i+1)*15)
        print "-"*i+"*"+"-"*(30-i)+" \r",
        

if __name__ == '__main__':
    
    # Log everything, and send it to stderr.
    logging.basicConfig(level=logging.INFO)
    
    UavtalkDemo()
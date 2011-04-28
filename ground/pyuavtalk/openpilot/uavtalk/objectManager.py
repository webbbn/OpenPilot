import logging
import sys
import os
import inspect

from openpilot.uavtalk.uavobject import *



class TimeoutException(Exception): 
    pass 

class ObjManager(object):
    
    def __init__(self, uavTalk, uavObjDefPath=None):
        self.objs = {}
        self.uavTalk = uavTalk
        if uavObjDefPath != None:
            self.importDefinitions(uavObjDefPath)
        uavTalk.setObjMan(self)
        
        
    def addObj(self, obj):
        self.objs[obj.objId] = obj
        
    def getObj(self, objId):
        try:
            return self.objs[objId]
        except KeyError:
            return None
        
    def getObjByName(self, name):
        for objId, obj in self.objs.items():
            if obj.name == name:
                return obj
        return None
        
    def importDefinitions(self, uavObjDefPath):
        logging.info("Importing UAVObject definitions from %s" % uavObjDefPath)
        sys.path.append(uavObjDefPath)
        for fileName in os.listdir(uavObjDefPath):
            if fileName[-3:] == ".py":
                logging.debug("Importing from file %s", fileName)
                module = __import__(fileName.replace(".py",""))
                for name in dir(module):
                    klass = getattr(module, name)
                    obj = getattr(module, name)
                    if inspect.isclass(obj):
                        if name != "UAVObject"  and name != "UAVMetaDataObject"  and name != "UAVDataObject"  and issubclass(klass, UAVObject):
                            logging.debug("Importing class %s", name)
                            obj = klass()
                            obj.name = name
                            setattr(self, name, obj)
                            self.addObj(obj)
                            metaObj = UAVMetaDataObject(obj.getMetaObjId())
                            obj.metadata = metaObj
                            metaObj.name = "Meta[%s]" % name
                            self.addObj(metaObj)
    
    def regObjectObserver(self, obj, observerObj, observerMethod):
        o = Observer(observerObj, observerMethod)
        obj.observers.append(o)
        
    def objUpdate(self, obj, rxData):
        obj.deserialize(rxData)
        obj.updateCnt += 1
        for observer in obj.observers:
            observer.call(obj)
        obj.updateEvent.acquire()
        obj.updateEvent.notifyAll()
        obj.updateEvent.release()
        
    def requestObjUpdate(self, obj):
        logging.debug("Requesting %s" % obj)
        self.uavTalk.sendObjReq(obj)
        
    def waitObjUpdate(self, obj, request=True, timeout=.5):
        logging.debug("Waiting for %s " % obj)
        cnt = obj.updateCnt
        if request:
            self.requestObjUpdate(obj)
        obj.updateEvent.acquire()
        obj.updateEvent.wait(timeout)
        obj.updateEvent.release()
        timeout = (cnt == obj.updateCnt)
        logging.debug("-> Waiting for %s Done. " % (obj))
        if timeout:
            s = "Timeout waiting for %s" % obj
            logging.debug(s)
            raise TimeoutException(s)
        
    def requestAllMetaDataUpdate(self):
        for objId, obj in self.objs.items():
            if obj.isMetaData():
                #print "GetMeta %s" % obj
                try:
                    self.waitObjUpdate(obj, request=True, timeout=.1)
                except TimeoutException:
                    #print "Timeout"
                    pass
                    
    def disableAllAutomaticUpdates(self):

        objsToExclude = [self.getObjByName("GCSTelemetryStats"), self.getObjByName("FlightTelemetryStats"), self.getObjByName("ObjectPersistence")]
        for i in xrange(len(objsToExclude)):
            objsToExclude[i] = objsToExclude[i].metadata.objId
            
        for objId, obj in self.objs.items():
            if obj.isMetaData() and obj.updateCnt>0:
                if obj.objId not in objsToExclude:
                    #print "Disabling automatic updates for %s" % (obj)
                    #print obj.telemetryUpdateMode.value
                    obj.telemetryUpdateMode.value = UAVMetaDataObject.UpdateMode.MANUAL
                    self.uavTalk.sendObject(obj)
                    
                    
    
class UavtalkDemo():
    
    #UAVOBJDEF_PATH = 'D:\Projects\Fred\OpenPilot\SVN\ground\uavobjgenerator\debug\python'
    UAVOBJDEF_PATH = "D:\\Projects\\Fred\\OpenPilot\\git_old\\OpenPilot\\build\\uavobject-synthetics\\python"
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
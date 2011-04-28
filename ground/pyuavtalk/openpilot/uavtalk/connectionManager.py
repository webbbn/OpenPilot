import logging
import serial
import sys

import objectManager
import uavobject
import time

class ConnectionManager(object):
    
    def __init__(self, uavTalk, objMan):
        self.uavTalk = uavTalk
        self.objMan = objMan
        self.connected = False
        
        self.flighttelemetrystats = sys.modules["flighttelemetrystats"]
        self.gcstelemetrystats = sys.modules["gcstelemetrystats"]
        self.ftsObj = self.objMan.getObj(self.flighttelemetrystats.FlightTelemetryStats.OBJID)
        self.gcsObj = self.objMan.getObj(self.gcstelemetrystats.GCSTelemetryStats.OBJID)
        
#        self.objMan.regObjectObserver(self.ftsObj, self, "_onFtsChange")
       
    def connect(self):
        timeout = True
        logging.debug("Connecting")
        startTime = time.clock()
        while not self.connected:
            try:
                self.objMan.waitObjUpdate(self.ftsObj, request=timeout, timeout=2)
                timeout = False
                self._onFtsChange()
                if self.connected:
                    self.objMan.waitObjUpdate(self.ftsObj.metadata)
                    self.ftsObj.metadata.telemetryUpdateMode.value = uavobject.UAVMetaDataObject.UpdateMode.PERIODIC
                    self.ftsObj.metadata.telemetryUpdatePeriod.value = 1000
                    self.uavTalk.sendObject(self.ftsObj.metadata)
                    self.objMan.regObjectObserver(self.ftsObj, self, "_onFtsChange")
                else:
                    pass
            except objectManager.TimeoutException:
                timeout = True
                self.connected = False
                logging.warning("Connecting TO")
                pass
        logging.debug("Connected in %.1fs" % (time.clock()-startTime))   
        
            
    def _onFtsChange(self, args=None):
        connected = False
        logging.debug("FTS State=%d TxFail=%3d RxFail=%3d TxRetry=%3d" % \
                     (self.ftsObj.Status.value, self.ftsObj.TxFailures.value, self.ftsObj.RxFailures.value, self.ftsObj.TxRetries.value))
        if self.ftsObj.Status.value == self.flighttelemetrystats.StatusField.DISCONNECTED:
            
            logging.debug(" Handshake REQ")
            self.gcsObj.Status.value = self.flighttelemetrystats.StatusField.HANDSHAKEREQ
            #obj.Status.value = flighttelemetrystats.StatusField.CONNECTED
            self.uavTalk.sendObject(self.gcsObj)    
            #uavTalk.sendObjReq(obj)
        elif self.ftsObj.Status.value == self.flighttelemetrystats.StatusField.HANDSHAKEACK:
            logging.debug(" Got Handshake ACK")
            self.gcsObj.Status.value = self.flighttelemetrystats.StatusField.CONNECTED
            self.uavTalk.sendObject(self.gcsObj)  
        elif self.ftsObj.Status.value == self.flighttelemetrystats.StatusField.CONNECTED:
            connected = True
            
        if self.connected:
            if not connected:
                logging.warning("DISCONNECTED")
        else:
            if connected:
                logging.debug("CONNECTED")
        
        self.connected = connected

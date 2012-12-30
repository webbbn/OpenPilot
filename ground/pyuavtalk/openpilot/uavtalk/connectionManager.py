##
##############################################################################
#
# @file       connectionManager.py
# @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2011.
# @brief      Base classes for python UAVObject
#   
# @see        The GNU Public License (GPL) Version 3
#
#############################################################################/
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
#



import logging
import serial
import sys

import objectManager
import uavobject
import uavtalk
import time

class ConnectionManager(object):
    
    def __init__(self, uavTalk, objMan):
        self.uavTalk = uavTalk
        self.objMan = objMan
        self.connected = False
        
        self.ftsObj = self.objMan.FlightTelemetryStats
        self.gcsObj = self.objMan.GCSTelemetryStats
        
        import openpilot.uavtalk.uavobjects.flighttelemetrystats as fts
        self.statusFieldClss = fts.StatusField
        
    def connect(self):
        '''
        The process for connecting is:
        GCSTelemetryStats.Status == STATUS_DISCONNECTED -> STATUS_HANDSHAKEREQ
        FlightTelemetryStats.Status == STATUS_DISCONNECTED -> STATUS_HANDSHAKEACK
        GCSTelemetryStats.Status == STATUS_HANDSHAKEREQ -> STATUS_CONNECTED
        FlightTelemetryStats.Status == STATUS_HANDSHAKEACK -> STATUS_CONNECTED
        '''
        timeout = True
        logging.info("Connecting")
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
                    self.ftsObj.metadata.updated()
                    self.objMan.regObjectObserver(self.ftsObj, self, "_onFtsChange")
                else:
                    pass
            except objectManager.TimeoutException:
                timeout = True
                self.connected = False
                logging.warning("Connecting TO")
                pass
        logging.info("Connected in %.1fs" % (time.clock()-startTime))

    def _onFtsChange(self, args=None):
        connected = False
        logging.debug("FTS State=%d TxFail=%3d RxFail=%3d TxRetry=%3d" % \
                     (self.ftsObj.Status.value, self.ftsObj.TxFailures.value, self.ftsObj.RxFailures.value, self.ftsObj.TxRetries.value))
        
        if self.ftsObj.Status.value == self.statusFieldClss.DISCONNECTED:  
            logging.debug(" Handshake REQ")
            self.gcsObj.Status.value = self.statusFieldClss.HANDSHAKEREQ
            self.gcsObj.updated()
            
        elif self.ftsObj.Status.value == self.statusFieldClss.HANDSHAKEACK:
            logging.debug(" Got Handshake ACK")
            self.gcsObj.Status.value = self.statusFieldClss.CONNECTED
            self.gcsObj.updated()
        
        elif self.ftsObj.Status.value == self.statusFieldClss.CONNECTED:
            connected = True
            
        if self.connected:
            if not connected:
                logging.warning("DISCONNECTED")
        else:
            if connected:
                logging.debug("CONNECTED")
        
        self.connected = connected

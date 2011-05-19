##
##############################################################################
#
# @file       example.py
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
import traceback
import sys

from openpilot.uavtalk.uavobject import *
from openpilot.uavtalk.uavtalk import *
from openpilot.uavtalk.objectManager import *
from openpilot.uavtalk.connectionManager import *
    
PORT = 29
UAVDEFPATH = "D:\Projects\Fred\OpenPilot\git\\build\uavobject-synthetics\python"



if __name__ == '__main__':

    try:
        serPort = serial.Serial(PORT, 57600, timeout=.5)
        uavTalk = UavTalk(serPort)
        objMan = ObjManager(uavTalk, UAVDEFPATH)
        uavTalk.start()
        
        while True:
            try:
                time.sleep(.5)
                objMan.StabilizationSettings.getUpdate()
                print objMan.StabilizationSettings.PitchPI.value
                break
            except TimeoutException:
                print "Timeout"
        
                                                    
        while True:
            try:
                # get update of ManualControlCommand 
                objMan.ManualControlCommand.getUpdate()  
                
                # calculate value out of Accessory1 input (-1 ... +1)
                txControl = objMan.ManualControlCommand.Accessory1.value
                value = 15 + txControl * 10
                objMan.StabilizationSettings.PitchPI.value[0] = value
                objMan.StabilizationSettings.updated()
                
                print "\r%-1.2f => %2.4f" % (txControl, value),
                time.sleep(.1)
                
            except TimeoutException:
                print "Timeout"
        
        print
        
    except KeyboardInterrupt:
        pass
    except Exception,e:
        print
        print "An error occured: ", e
        print
        traceback.print_exc()
    
    try:
        uavTalk.stop()
    except Exception:
        pass
    
    raw_input("Press ENTER, the application will close")



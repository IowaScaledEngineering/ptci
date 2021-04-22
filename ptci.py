# *************************************************************************
# Title:    Protothrottle Computer Interface
# Authors:  Nathan D. Holmes <maverick@drgw.net>
#           Michael D. Petersen <railfan@drgw.net>
# File:     ptci.py
# License:  GNU General Public License v3
#
# LICENSE:
#   Copyright (C) 2021 Nathan Holmes & Michael Petersen
#    
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 3 of the License, or
#   any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
# DESCRIPTION:
#
#*************************************************************************

import sys
import time
import traceback
import argparse
import re
import struct
import mrbus
from mrbus import packet
import datetime
import json
import logging
import sys, os, time

from protothrottle import protothrottle

try:
  import serial.tools.list_ports
except ImportError:
  raise ImportError('serial.tools.list_ports is missing - you probably need to use pip to install serial and pySerial')

def findXbeePort():
  """This looks for the first USB serial port with an FTDI bridge chip.  In the RasPi embedded esu-bridge, this will always be the XBee."""
  ports = list(serial.tools.list_ports.grep("ttyUSB"))
  for p in ports:
    if "FTDI" == p.manufacturer:
      return p.device
  return None
   

def main(mainParms):
  # Unpack incoming parameters
  # mainParms = {'startupDirectory': pwd, 'configFile': configFile, 'serialPort': args.serial, 'isDaemon':isDaemon, 'logFile':args.logfile }

  throttleLetter = mainParms['throttle'][0:1]
  if ord(throttleLetter) not in range(ord('A'), ord('Z')+1):
    print("ERROR: Invalid throttle letter '%c'" % (throttleLetter))
    return

  mode = mainParms['mode']

  throttleAddr = 0x30 + (ord(throttleLetter) - ord('A'))
  
  slot = int(mainParms['slot'])
  if slot > 20 or slot < 1:
    print("ERROR: Slot num %d out of range 1-20, aborting." % (slot))
    return


  serialPort = mainParms['serialPort']
  logger = mainParms['logger']
  mrbee = None

  # Initialize MRBus / MRBee client if necessary
  if mrbee is None:
    try:
      # If we didn't get the port from either of those, search for an FTDI bridge
      #   part using findXbeePort()
      if serialPort is None:
        serialPort = findXbeePort()

      if serialPort is None:
        logger.warning("No XBee/MRBus interface found, waiting and retrying...")

      mrbee = mrbus.mrbus(serialPort, 0xFE, logger=logger, busType='mrbee')
      mrbee.setXbeeLED('D9', True);
              
    except(KeyboardInterrupt):
      raise
    except Exception as e:
      if mrbee is not None:
        mrbee.disconnect()
        mrbee = None
        logger.exception("Exception in starting MRBus interface")

  print("MRBus Connection Established")

  pt = protothrottle(mrbee)
  pt.attach(throttleLetter)

  print("PT Attached")
  print("Global Configuration:\n----------------------------------")
  pt.readGlobalConfig()
  print(pt.globalConfig)

  # Okay, what are we supposed to do here?
  if mode == 'read':
    slotNum = int(args.config)
    print("\nSlot %d Configuration:\n----------------------------------" %(slotNum))
    pt.readSlot(slotNum)
    print(pt.slots[0])

if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("-s", "--serial", help="specify serial device for XBee radio", type=str, default='/dev/ttyUSB0')
  ap.add_argument("-t", "--throttle", help="throttle ID, A-Z", type=str, default='A')
  ap.add_argument("-c", "--config", help="config slot number", type=int, default=1)
  ap.add_argument("-r", "--read", help="read throttle to file", type=str, default=None)
  ap.add_argument("-w", "--write", help="write file to throttle", type=str, default=None)
  args = ap.parse_args()
  logger = logging.getLogger('main')
  # Because we might become a daemon, we need to canonicalize our path to our configuration file
  mainParms = {'serialPort': args.serial, 'logger':logger, 'throttle':args.throttle, 'slot':args.config, 'readToFile':args.read, 'writeToThrottle':args.write, 'mode':'read'}

  try:
    main(mainParms)
  except Exception as e:
    print(e)
    traceback.print_exc()
  logger.info("Terminated")
  logging.shutdown()

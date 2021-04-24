import mrbus
from mrbus import packet
import time
import struct

def readEEPResponseValidate(txPkt, rxPkt):
  if len(rxPkt.data) == (txPkt.data[2] + 2) and (rxPkt.data[0] + (rxPkt.data[1]*256)) == (txPkt.data[0] + (txPkt.data[1]*256)):
    return True
  return False

def writeEEPResponseValidate(txPkt, rxPkt):
  if rxPkt.data == txPkt.data:
    return True

  return False

class protothrottle(object):
  def __init__(self, bus):
    self.bus = bus
    self.throttleID = 'A'
    self.throttleAddr = 0x30
    self.version = None
    self.slots = [{ } for i in range(20)]
    self.globalConfig = { }
    self.interpreter = None


  def writeMemory(self, bus, targetAddr, memStart, data):
    bytesWritten = 0
    memLen = len(data)
    
    # 'W' can handle up to 12 data bytes at a time
    while bytesWritten < memLen:
      bytesToWrite = memLen    
      if memLen >= 12:
        bytesToWrite = 12

      txPkt = packet(targetAddr, bus.addr, ord('W'), [memStart & 0xFF, memStart>>8] + data[bytesWritten:bytesWritten+bytesToWrite])
      rxPkt = self.mrbusReliableTransact(bus, txPkt, responseValidate=writeEEPResponseValidate)

      if None == rxPkt:
        # apparently we timed out
        raise IOError('timeout')

      memStart = memStart + bytesToWrite
      bytesWritten = bytesWritten + bytesToWrite
      
    
  def readMemory(self, bus, targetAddr, memStart, memLen):
    bytesRead = 0
    data = []
    
    # 'R' can handle up to 12 data bytes at a time
    while bytesRead < memLen:
      bytesToRead = memLen    
      if memLen >= 12:
        bytesToRead = 12

      txPkt = packet(targetAddr, bus.addr, ord('R'), [memStart & 0xFF, memStart>>8, bytesToRead])
      rxPkt = self.mrbusReliableTransact(bus, txPkt, responseValidate=readEEPResponseValidate)

      if None == rxPkt:
        # apparently we timed out
        raise IOError('timeout')

      for i in range(2, 2+bytesToRead):
        data.append(rxPkt.data[i])

      memStart = memStart + bytesToRead
      bytesRead = bytesRead + bytesToRead
      
    return bytes(data)

  def mrbusReliableTransact(self, bus, txPkt, responseValidate=None, timeout=0.3, retries=5):
    retry = retries
    response = None

    while retry > 0:
      bus.sendpkt(txPkt.dest, [txPkt.cmd] + txPkt.data, txPkt.src)
      now = start = time.time()
      response = None

      while now - start <= timeout:
        rxPkt = bus.getpkt()
        now = time.time()
        if None == rxPkt:
          continue
            
        #print("Got packet - src = 0x%02X, dest=0x%02X, cmd='%c', len(data)=%d" % (rxPkt.src, rxPkt.dest, rxPkt.cmd, len(rxPkt.data)))
        #print(["0x%02X" % d for d in rxPkt.data])
        
        if rxPkt.dest != txPkt.src or rxPkt.src != txPkt.dest or rxPkt.cmd != (txPkt.cmd + 32): 
          continue
        
        if None != responseValidate and not responseValidate(txPkt, rxPkt):
          continue
          
        response = rxPkt
        break

      if None == response:
        retry = retry - 1
        continue

      break

    return response

    
    
  def readVersion(self, targetAddr):
    txPkt = packet(targetAddr, self.bus.addr, ord('V'), [])
    rxPkt = self.mrbusReliableTransact(self.bus, txPkt)
    
    if None == rxPkt:
      print("ERROR: Throttle not found")
      return None

    if len(rxPkt.data) < 9:
      print("ERROR: Version packet in unexpected form")
      return None
    
    if rxPkt.data[6] != ord('C') or rxPkt.data[7] != ord('S') or rxPkt.data[8] != ord('T'):
      print("ERROR: Address 0x%02X is not a ProtoThrottle" % targetAddr)
      return None

    if len(rxPkt.data) >= 12:
      version = (rxPkt.data[9],rxPkt.data[10],rxPkt.data[11])
    else:
      version = (1,0,0)

    print("Throttle HW version %d.%d - Firmware version %d.%d.%d git rev: %06x" % (rxPkt.data[4], rxPkt.data[5], version[0], version[1], version[2], 0xFFFFFF & ((rxPkt.data[1]<<16) + (rxPkt.data[2]<<8) + rxPkt.data[3])) )

    mem = self.readMemory(self.bus, targetAddr, 0x0E, 2)
    # The first versions of firmware didn't put an EEP layout number in place
    if 0xFF == mem[0] and 0xFF == mem[1]:
      mem[0] = 1
      mem[1] = 0
    print("Throttle reports eeprom layout %d.%d" % (mem[0], mem[1]) )

    response = {
      'hw_major':rxPkt.data[4],
      'hw_minor':rxPkt.data[5],
      'eep_major':mem[0],
      'eep_minor':mem[1],
      'version':version,
      'gitrev': '%06x' % ((rxPkt.data[1]<<16) + (rxPkt.data[2]<<8) + rxPkt.data[3])
    }

    return response


  def attach(self, throttleID):
    self.throttleID = throttleID
    self.throttleAddr = 0x30 + (ord(self.throttleID) - ord('A'))
    self.version = self.readVersion(self.throttleAddr)
    
    if self.version == None:
      raise IOError('Throttle Not Responding')
    
    if self.version['version'] < (1,2,0):
      raise IOError('Throttle version too low')
    
    
    # Based on version, attach the correct interpreter
    self.interpreter = pt_interpreter_v12()
    self.interpreter.help()
    self.slots = [{ } for i in range(20)]
    self.globalConfig = { }



  def readFromThrottle(self):
    self.readGlobalConfig()
    for slotNum in range(0,20):
      self.readSlot(slotNum)


  def readGlobalConfig(self):
    memStart = 0x10
    memLen = 0x70
    confmem = self.readMemory(self.bus, self.throttleAddr, self.interpreter.globalConfMemStart, self.interpreter.globalConfMemLen)
    self.globalConfig = self.interpreter.readGlobalConfig(confmem)

  def readSlot(self, slotNum):
    if slotNum > self.interpreter.numConfigSlots:
      return

    slotmem = self.readMemory(self.bus, self.throttleAddr, self.interpreter.getSlotStart(slotNum), self.interpreter.getSlotLen(slotNum))
    self.slots[slotNum-1] = self.interpreter.readSlotConfig(slotmem)

  def getSlot(self, slotNum):
    if slotNum > self.interpreter.numConfigSlots:
      return { }
    return self.slots[slotNum-1]

    
    
  def writeGlobalConfig(self):
    memStart = 0x10
    memLen = 0x70
    # Read raw throttle configuration memory
    # confmem = readMemory(mrbee, throttleAddr, 0x10, memLen)
    # Canonicalize memory into dictionary to fit in global conf
    # self.globalConfig = { } # FIXME

  def setSlot(self, slotNum, conf):
    if slotNum > self.interpreter.numConfigSlots:
      return
    self.slots[slotNum-1] = conf


  def writeSlot(self, slotNum):
    if slotNum > self.interpreter.numConfigSlots:
      return

    slotmem = self.interpreter.writeSlotConfig(self.slots[slotNum-1])
    self.writeMemory(self.bus, self.throttleAddr, self.interpreter.getSlotStart(slotNum), slotmem)

  
class pt_interpreter_v12(object):

  def __init__(self):
    self.numConfigSlots     = 20
    self.slotConfMemStart   = 0x80
    self.slotConfMemLen     = 0x80
    self.globalConfMemLen   = 0x16
    self.globalConfMemStart = 0x10
    self.functionXlate = {
      'F00_MOM':0x00,
      'F01_MOM':0x01,
      'F02_MOM':0x02,
      'F03_MOM':0x03,
      'F04_MOM':0x04,
      'F05_MOM':0x05,
      'F06_MOM':0x06,
      'F07_MOM':0x07,
      'F08_MOM':0x08,
      'F09_MOM':0x09,
      'F10_MOM':0x0A,
      'F11_MOM':0x0B,
      'F12_MOM':0x0C,
      'F13_MOM':0x0D,
      'F14_MOM':0x0E,
      'F15_MOM':0x0F,
      'F16_MOM':0x10,
      'F17_MOM':0x11,
      'F18_MOM':0x12,
      'F19_MOM':0x13,
      'F20_MOM':0x14,
      'F21_MOM':0x15,
      'F22_MOM':0x16,
      'F23_MOM':0x17,
      'F24_MOM':0x18,
      'F25_MOM':0x19,
      'F26_MOM':0x1A,
      'F27_MOM':0x1B,
      'F28_MOM':0x1C,
      'F00_LAT':0x40,
      'F01_LAT':0x41,
      'F02_LAT':0x42,
      'F03_LAT':0x43,
      'F04_LAT':0x44,
      'F05_LAT':0x45,
      'F06_LAT':0x46,
      'F07_LAT':0x47,
      'F08_LAT':0x48,
      'F09_LAT':0x49,
      'F10_LAT':0x4A,
      'F11_LAT':0x4B,
      'F12_LAT':0x4C,
      'F13_LAT':0x4D,
      'F14_LAT':0x4E,
      'F15_LAT':0x4F,
      'F16_LAT':0x50,
      'F17_LAT':0x51,
      'F18_LAT':0x52,
      'F19_LAT':0x53,
      'F20_LAT':0x54,
      'F21_LAT':0x55,
      'F22_LAT':0x56,
      'F23_LAT':0x57,
      'F24_LAT':0x58,
      'F25_LAT':0x59,
      'F26_LAT':0x5A,
      'F27_LAT':0x5B,
      'F28_LAT':0x5C,
      'FN_OFF':0x80,
      'FN_EMRG':0x81,
      'FN_BRKTEST':0xC0 }


  def getSlotStart(self, slotNum):
    return self.slotConfMemStart + (slotNum-1) * self.slotConfMemLen

  def getSlotLen(self, slotNum):
    return self.slotConfMemLen

  def funcToName(self, funcVal):
    for name,val in self.functionXlate.items():
      if funcVal == val:
        return name
    return 'FN_OFF'

  def funcNameToValue(self, configSlotValues, name):
    fn = 'FN_OFF'
    if name in configSlotValues.keys() and configSlotValues[name] in self.functionXlate.keys():
      fn = configSlotValues[name]

    return self.functionXlate[fn]

  def readGlobalConfig(self, mem):
    globalConfValues = { }
    
    globalConfValues['sleepTimeMinutes'] = mem[0x01]
    globalConfValues['fcDeadReckoningTime'] = mem[0x02]
    globalConfValues['batteryOkayVolts'] = "%.1f" % (mem[0x04] / 10.0)
    globalConfValues['batteryWarnVolts'] = "%.1f" % (mem[0x05] / 10.0)
    globalConfValues['batteryCriticalVolts'] = "%.1f" % (mem[0x06] / 10.0)
    globalConfValues['transmitHoldoffSeconds'] = "%.2f" % (mem[0x0D] / 100.0)
    globalConfValues['timeSourceAddress'] = mem[0x0E]
    globalConfValues['receiverAddress'] = mem[0x0F]
    
    globalConfValues['hornThreshold'] = mem[0x10]
    globalConfValues['brakeThreshold'] = mem[0x11]
    globalConfValues['brakeLowLimit'] = mem[0x12]
    globalConfValues['brakeHighLimit'] = mem[0x13]
    globalConfValues['brakePressureConfig'] = mem[0x14]
    globalConfValues['alerterTimeoutSeconds'] = mem[0x15]
    
    return globalConfValues

  def writeSlotConfig(self, configSlotValues):
    mem = [0xFF for i in range(self.slotConfMemLen)]

    # Clean up the address and handle the case where it's either not all there
    # or doesn't make any sense
    addrVal = 3
    addrType = 'short'
    if 'address' in configSlotValues.keys():
      try: 
        addrVal = max(1,min(9999,int(configSlotValues['address'])))
      except:
        addrVal = 3
    
    if 'addressType' in configSlotValues.keys():
      addrType = configSlotValues['addressType']

    if addrType == 'short' and addrVal > 127:
      addrType = 'long'

    if addrType == 'short':
      addrVal = 0x8000 | (0x7FFF & addrVal)
    
    b = struct.pack("<H", addrVal)
    mem[0x00] = b[0]
    mem[0x01] = b[1]
    mem[0x02] = self.funcNameToValue(configSlotValues, 'funcHorn')
    mem[0x03] = self.funcNameToValue(configSlotValues, 'funcBell')
    mem[0x04] = self.funcNameToValue(configSlotValues, 'funcBrake')
    mem[0x13] = self.funcNameToValue(configSlotValues, 'funcBrakeOff')
    mem[0x05] = self.funcNameToValue(configSlotValues, 'funcAuxButton')
    
    mem[0x06] = self.funcNameToValue(configSlotValues, 'funcEngineOn')
    mem[0x07] = self.funcNameToValue(configSlotValues, 'funcEngineOff')
    mem[0x0A] = self.funcNameToValue(configSlotValues, 'funcFrontHeadlight')
    mem[0x08] = self.funcNameToValue(configSlotValues, 'funcFrontDim1')
    mem[0x09] = self.funcNameToValue(configSlotValues, 'funcFrontDim2')
    
    mem[0x0B] = self.funcNameToValue(configSlotValues, 'funcFrontDitchlight')
    mem[0x0E] = self.funcNameToValue(configSlotValues, 'funcRearHeadlight')
    mem[0x0C] = self.funcNameToValue(configSlotValues, 'funcRearDim1')
    mem[0x0D] = self.funcNameToValue(configSlotValues, 'funcRearDim2')
    mem[0x0F] = self.funcNameToValue(configSlotValues, 'funcRearDitchlight')


    mem[0x10] = self.funcNameToValue(configSlotValues, 'funcUpButton')
    mem[0x11] = self.funcNameToValue(configSlotValues, 'funcDownButton')
    mem[0x12] = self.funcNameToValue(configSlotValues, 'funcThrottleUnlock')
    mem[0x14] = self.funcNameToValue(configSlotValues, 'funcReverserSwap')
    mem[0x30] = self.funcNameToValue(configSlotValues, 'funcCompressor')

    mem[0x31] = self.funcNameToValue(configSlotValues, 'funcBrakeTest')
    mem[0x32] = self.funcNameToValue(configSlotValues, 'funcReverserCentered')
    mem[0x33] = self.funcNameToValue(configSlotValues, 'funcAlerter')

    # Set notch slots to default values
    mem[0x20] = 7;
    mem[0x21] = 23;
    mem[0x22] = 39;
    mem[0x23] = 55;
    mem[0x24] = 71;
    mem[0x25] = 87;
    mem[0x26] = 103;
    mem[0x27] = 119;

    for notch in range (1,9):
      notchName = "notch%dSpeed" % notch
      if notchName in configSlotValues.keys():
        try:
          mem[0x20 + notch - 1] = max(1, min(126, int(configSlotValues[notchName])))
        except:
          pass

    # Forced On functions 0x18-0x1B as bitmask
    fon = 0
    for bit in range(0, 29):
      if ("F%02d" % (bit)) in configSlotValues['forceOn']:
        fon = fon | (1<<bit)
    b = struct.pack("<I", fon)
    for i in range(0, 4):
      mem[0x18 + i] = b[i]
    
    # Forced Off functions 0x1C-0x1F as bitmask
    foff = 0
    for bit in range(0, 29):
      if ("F%02d" % (bit)) in configSlotValues['forceOff']:
        foff = foff | (1<<bit)
    b = struct.pack("<I", foff)
    for i in range(0, 4):
      mem[0x1C + i] = b[i]

    if 'brakePulseWidthMilliseconds' in configSlotValues.keys():
      mem[0x16] = max(2, min(10, int(configSlotValues['brakePulseWidthMilliseconds']) / 100.0))

    options = 0x01

    if 'optionEStopOnBrake' in configSlotValues.keys():
      if configSlotValues['optionEStopOnBrake'] == "On":
        options |= 0x01
      else:
        options &= ~(0x01)

    if 'optionReverserSwap' in configSlotValues.keys():
      if configSlotValues['optionReverserSwap'] == "On":
        options |= 0x02

    if 'optionVariableBrake' in configSlotValues.keys():
      if configSlotValues['optionVariableBrake'] == "On":
        options |= 0x04
        
    if 'optionSteppedBrake' in configSlotValues.keys():
      if configSlotValues['optionSteppedBrake'] == "On":
        options |= 0x08
        
    mem[0x17] = options

    return mem

   
  def readSlotConfig(self, mem):
    configSlotValues = { }
    
    addrVal = struct.unpack("<H", mem[0x00:0x02])[0]
    if (addrVal & 0x8000):
      configSlotValues['addressType'] = 'short'
      configSlotValues['address'] = addrVal & 0x7FFF
    else:
      configSlotValues['addressType'] = 'long'
      configSlotValues['address'] = addrVal & 0x7FFF

    configSlotValues['funcHorn'] = self.funcToName(mem[0x02])
    configSlotValues['funcBell'] = self.funcToName(mem[0x03])
    configSlotValues['funcBrake'] = self.funcToName(mem[0x04])
    configSlotValues['funcBrakeOff'] = self.funcToName(mem[0x13])
    configSlotValues['funcAuxButton'] = self.funcToName(mem[0x05])
    configSlotValues['funcEngineOn'] = self.funcToName(mem[0x06])
    configSlotValues['funcEngineOff'] = self.funcToName(mem[0x07])
    configSlotValues['funcFrontHeadlight'] = self.funcToName(mem[0x0A])
    configSlotValues['funcFrontDim1'] = self.funcToName(mem[0x08])
    configSlotValues['funcFrontDim2'] = self.funcToName(mem[0x09])
    configSlotValues['funcFrontDitchlight'] = self.funcToName(mem[0x0B])
    configSlotValues['funcRearHeadlight'] = self.funcToName(mem[0x0E])
    configSlotValues['funcRearDim1'] = self.funcToName(mem[0x0C])
    configSlotValues['funcRearDim2'] = self.funcToName(mem[0x0D])
    configSlotValues['funcRearDitchlight'] = self.funcToName(mem[0x0F])

    configSlotValues['funcUpButton'] = self.funcToName(mem[0x10])
    configSlotValues['funcDownButton'] = self.funcToName(mem[0x11])
    configSlotValues['funcThrottleUnlock'] = self.funcToName(mem[0x12])
    configSlotValues['funcReverserSwap'] = self.funcToName(mem[0x14])

    configSlotValues['funcCompressor'] = self.funcToName(mem[0x30])
    configSlotValues['funcBrakeTest'] = self.funcToName(mem[0x31])
    configSlotValues['funcReverserCentered'] = self.funcToName(mem[0x32])
    configSlotValues['funcAlerter'] = self.funcToName(mem[0x33])
    
    configSlotValues['notch1Speed'] = "%d" % mem[0x20]
    configSlotValues['notch2Speed'] = "%d" % mem[0x21]
    configSlotValues['notch3Speed'] = "%d" % mem[0x22]
    configSlotValues['notch4Speed'] = "%d" % mem[0x23]
    configSlotValues['notch5Speed'] = "%d" % mem[0x24]
    configSlotValues['notch6Speed'] = "%d" % mem[0x25]
    configSlotValues['notch7Speed'] = "%d" % mem[0x26]
    configSlotValues['notch8Speed'] = "%d" % mem[0x27]
    
    # Forced On functions 0x18-0x1B as bitmask
    configSlotValues['forceOn'] = []
    fon = struct.unpack("<I", mem[0x18:0x1C])[0]
    for bit in range(0, 29):
      if fon & (1<<bit):
        configSlotValues['forceOn'].append("F%02d" % (bit))
    
    # Forced Off functions 0x1C-0x1F as bitmask
    configSlotValues['forceOff'] = []
    foff = struct.unpack("<I", mem[0x1C:0x20])[0]
    for bit in range(0, 29):
      if foff & (1<<bit):
        configSlotValues['forceOff'].append("F%02d" % (bit))
    
    configSlotValues['brakePulseWidthMilliseconds'] = "%d" % (mem[0x16] * 100)
    
    if mem[0x17] & 0x01:
      configSlotValues['optionEStopOnBrake'] = "On"
    else:
      configSlotValues['optionEStopOnBrake'] = "Off"

    if mem[0x17] & 0x02:
      configSlotValues['optionReverserSwap'] = "On"
    else:
      configSlotValues['optionReverserSwap'] = "Off"

    if mem[0x17] & 0x04:
      configSlotValues['optionVariableBrake'] = "On"
    else:
      configSlotValues['optionVariableBrake'] = "Off"

    if mem[0x17] & 0x08:
      configSlotValues['optionSteppedBrake'] = "On"
    else:
      configSlotValues['optionSteppedBrake'] = "Off"
    
    return configSlotValues

  def help(self):
    print("Loading interpreter for eeprom version 1.1")

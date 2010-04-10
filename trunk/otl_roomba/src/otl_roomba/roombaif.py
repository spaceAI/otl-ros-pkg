#!/usr/bin/env python
import serial
import time
import struct

#
# command define
#
STARTCMD = 128
BAUDCMD = 129
CONTROLCMD = 130  # deplicated
SAFECMD = 131
FULLCMD = 132
POWERCMD = 133
SPOTCMD = 134
CLEANCMD = 135
MAXCMD = 136
DRIVECMD = 137
MOTORSCMD = 138
LEDSCMD = 139
DOCKCMD = 143
DEGITCMD = 163
ASCIICMD = 164

#
# velocity define
#
MAXVEL = 500
MINVEL = -500

#
# radius define
#
STRAIGHTRADIUS = 0x7FFF
CWRADIUS = -1
CCWRADIUS = 1
MAXRADIUS = 2000
MINRADIUS = -2000

#
# LED define
#
DEBRISLED = 0x1
SPOTLED = 0x2
DOCKLED = 0x4
CHECKLED = 0x8

FWDLED = SPOTLED
BWDLED = DOCKLED
LEFTLED = CHECKLED
RIGHTLED = DEBRISLED

#
# serial define
#
DEFAULT_DEVICE = '/dev/ttyUSB0'
DEFAULT_BAUD = 115200


class RoombaIf(object):
    """
    control interface for Roomba OI.
    this soft use usb serial port for controlling the robot.
    """
    def __init__(self, full=False):
        self._opened = False
        self._full = False

    def setup(self, dev=DEFAULT_DEVICE, otl=True):
        if self.open(dev):
            self.activate(full=self._full)
            self.setLed(DEBRISLED, 0, 0)
            self.setLed(CHECKLED, 0, 50)
            self.setLed(DOCKLED, 0, 100)
            self.setLed(SPOTLED, 0, 150)
            self.setLed(DEBRISLED | CHECKLED | DOCKLED | SPOTLED, 0, 200)
            self.setLed(DEBRISLED | CHECKLED | DOCKLED | SPOTLED, 100, 200)
            if otl:
                self.setOTL()

    def setdown(self):
        self._sendCommand(POWERCMD)
        self.close()

    def open(self, dev=DEFAULT_DEVICE):
        try:
            self.ser_ = serial.Serial(dev,
                                      baudrate=DEFAULT_BAUD,
                                      timeout=0.1)
        except serial.SerialException:
            print('serial open error')
            return False
        self.ser_.open()
        self._opened = True
        return True

    def close(self):
        if self._opened:
            self.ser_.close()

    def _sendCommand(self, cmd):
        if isinstance(cmd, int):
            cmd = struct.pack('B', cmd)
        if isinstance(cmd, str):
            if self._opened:
                try:
                    self.ser_.write(cmd)
                except serial.SerialTimeoutException, val:
                    print 'write error'
            else:
                print '[debug]%s' % cmd
            time.sleep(0.1)
        else:
            print ('_sendCommand error')

    def setVel(self, vel, radius):
        if vel > MAXVEL:
            print ('speed limit over %d > %d' % (vel, MAXVEL))
            vel = MAXVEL
        elif vel < MINVEL:
            print ('speed limit over %d < %d' % (vel, MINVEL))
            vel = MINVEL
        if radius != STRAIGHTRADIUS and radius > MAXRADIUS:
            print ('radius limit over %d > %d' % (radius, MAXRADIUS))
            radius = STRAIGHTRADIUS
        elif vel < MINRADIUS:
            print ('radius limit over %d < %d' % (radius, MINRADIUS))
            radius = STRAIGHTRADIUS
        cmd = struct.pack('>Bhh', DRIVECMD, vel, radius)
        self._sendCommand(cmd)

    def cleanOff(self):
        cmd = struct.pack('BB', MOTORSCMD, 0)
        self._sendCommand(cmd)

    def cleanOn(self):
        cmd = struct.pack('BB', MOTORSCMD, 7)
        self._sendCommand(cmd)

    def setLed(self, bits, color, power):
        cmd = struct.pack('>BBBB', LEDSCMD, bits, color, power)
        self._sendCommand(cmd)

    def setDegit(self, d1, d2, d3, d4):
        cmd = struct.pack('>BBBBB', DEGITCMD, d1, d2, d3, d4)
        self._sendCommand(cmd)

    def setOTL(self):
        self.setDegit(63, 7, 1, 56)

    def setAscii(self, ltr):
        cmd = struct.pack('>Bcccc', ASCIICMD, ltr[0], ltr[1], ltr[2], ltr[3])
        self._sendCommand(cmd)

    def goFwd(self, speed):
        self.setLed(FWDLED, 255, 100)
        self.setVel(speed, STRAIGHTRADIUS)

    def goBwd(self, speed):
        self.setLed(BWDLED, 0, 100)
        self.setVel(-speed, STRAIGHTRADIUS)

    def goRight(self, speed):
        self.setLed(RIGHTLED, 255, 100)
        self.setVel(speed, CWRADIUS)

    def goLeft(self, speed):
        self.setLed(LEFTLED, 255, 100)
        self.setVel(speed, CCWRADIUS)

    def goStop(self):
        self.setLed(0, 255, 100)
        self.setVel(0, CCWRADIUS)

    def activate(self, full=False):
        self._sendCommand(STARTCMD)
        self._sendCommand(SAFECMD)
        if full:
            self._sendCommand(FULLCMD)


if __name__ == '__main__':
    rmb = RoombaIf()
    rmb.setup()
    rmb.goFwd(100)
    time.sleep(2)
    rmb.goFwd(200)
    time.sleep(2)
    rmb.goRight(100)
    time.sleep(3)
    rmb.goFwd(200)
    time.sleep(1)
    rmb.goBwd(200)
    time.sleep(1)
    rmb.goFwd(200)
    time.sleep(1)
    rmb.goStop()
    rmb.setdown()

__all__ = [
    'MAXVEL',
    'MINVEL',
    'STRAIGHTRADIUS',
    'CWRADIUS',
    'CCWRADIUS',
    'MAXRADIUS',
    'MINRADIUS',
    'DEBRISLED',
    'SPOTLED',
    'DOCKLED',
    'CHECKLED',
    
    'FWDLED',
    'BWDLED',
    'LEFTLED',
    'RIGHTLED',
    'RoombaIf'
    ]

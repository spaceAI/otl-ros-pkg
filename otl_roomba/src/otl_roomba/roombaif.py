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

COMMAND_LIST = [STARTCMD, BAUDCMD, CONTROLCMD, SAFECMD, FULLCMD,
                POWERCMD, SPOTCMD, CLEANCMD, MAXCMD, DRIVECMD, 
                MOTORSCMD, LEDSCMD, DOCKCMD, DEGITCMD, ASCIICMD]

#
# velocity define
#
MAXVEL = 500
MINVEL = -500

#
# radius define
#
STRAIGHTRAD = 0x7FFF
CWRAD = -1
CCWRAD = 1
MAXRAD = 2000
MINRAD = -2000

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

class RoombaError(Exception):
    pass

class RoombaIf(object):
    """
    control interface for Roomba OI.
    this soft use usb serial port for controlling the robot.
    """
    def __init__(self, full=False, debug=False, device=DEFAULT_DEVICE):
        self._opened = False
        self._full = False
        self._debug = debug
        self._device = device

    def setup(self, otl=True):
        self.open(self._device)
        self.activate(full=self._full)
        self.set_led(DEBRISLED, 0, 0)
        self.set_led(CHECKLED, 0, 50)
        self.set_led(DOCKLED, 0, 100)
        self.set_led(SPOTLED, 0, 150)
        self.set_led(DEBRISLED | CHECKLED | DOCKLED | SPOTLED, 0, 200)
        self.set_led(DEBRISLED | CHECKLED | DOCKLED | SPOTLED, 100, 200)
        if otl:
            self.set_otl()

    def teardown(self):
        self._send_command(POWERCMD)
        # anyway close
        self.close()

    def open(self, dev=DEFAULT_DEVICE):
        if not self._debug:
            try:
                self.ser_ = serial.Serial(dev,
                                          baudrate=DEFAULT_BAUD,
                                          timeout=0.1)
                self.ser_.open()
            except serial.SerialException:
                raise RoombaError('serial initialize error')
            self._opened = True

    def close(self):
        if self._opened:
            self.ser_.close()

    def _send_command(self, cmd):
        if not self._debug and not self._opened:
            raise RoombaError('not opened')
        if isinstance(cmd, int) and cmd in COMMAND_LIST:
            try:
                cmd = struct.pack('B', cmd)
            except struct.error:
                raise RoombaError('command error')
        if isinstance(cmd, str):
            if not self._debug:
                try:
                    self.ser_.write(cmd)
                except serial.SerialTimeoutException:
                    self.close()
                    raise RoombaError('serial write timeout error')
                except ValueError:
                    self.close()
                    raise RoombaError('serial write error')
            else:
                print '[debug]%s' % cmd
            time.sleep(0.05)
        else:
            raise RoombaError('command type error')

    def set_vel(self, vel, rad):
        if not isinstance(vel, int) or not isinstance(rad, int):
            raise RoombaError('set vel inval error')
        if vel > MAXVEL:
            print ('speed limit over %d > %d' % (vel, MAXVEL))
            vel = MAXVEL
        elif vel < MINVEL:
            print ('speed limit over %d < %d' % (vel, MINVEL))
            vel = MINVEL
        if rad != STRAIGHTRAD and rad > MAXRAD:
            print ('rad limit over %d > %d' % (rad, MAXRAD))
            rad = STRAIGHTRAD
        elif rad < MINRAD:
            print ('rad limit over %d < %d' % (rad, MINRAD))
            rad = STRAIGHTRAD
        try:
            cmd = struct.pack('>Bhh', DRIVECMD, vel, rad)
        except struct.error:
            raise RoombaError('set vel inval error')
        self._send_command(cmd)
        return (vel, rad)

    def clean_off(self):
        try:
            cmd = struct.pack('BB', MOTORSCMD, 0)
        except:
            raise RoombaError('Fatal error in clean_off')
        self._send_command(cmd)

    def clean_on(self):
        try:
            cmd = struct.pack('BB', MOTORSCMD, 7)
        except struct.error:
            raise RoombaError('Fatal error in clean_off')
        self._send_command(cmd)

    def set_led(self, bits, color, power):
        if not (0 <= bits <= 255 and
                0 <= color <= 255 and
                0 <= power <= 255):
            raise RoombaError('inval error')
        try:
            cmd = struct.pack('>BBBB', LEDSCMD, bits, color, power)
        except struct.error:
            raise RoombaError('inval error in set_led')
        self._send_command(cmd)

    def set_degit(self, d1, d2, d3, d4):
        if not (0 <= d1 <= 255 and
                0 <= d2 <= 255 and
                0 <= d3 <= 255 and
                0 <= d4 <= 255):
            raise RoombaError('out of range')
        try:
            cmd = struct.pack('>BBBBB', DEGITCMD, d1, d2, d3, d4)
        except struct.error:
            raise RoombaError('inval error in set_degit')
        return self._send_command(cmd)

    def set_otl(self):
        self.set_degit(63, 7, 1, 56)

    def set_ascii(self, ltr):
        if isinstance(ltr, basestring) and len(ltr) == 4:
            try:
                cmd = struct.pack('>Bcccc', ASCIICMD, ltr[0], ltr[1], ltr[2], ltr[3])
            except struct.error:
                raise RoombaError('inval error in set_ascii')
        else:
            raise RoombaError('inval type error in set_ascii')
        self._send_command(cmd)

    def activate(self, full=False):
        self._send_command(STARTCMD)
        self._send_command(SAFECMD)
        if full:
            self._send_command(FULLCMD)

__all__ = [
    'MAXVEL',
    'MINVEL',

    'STRAIGHTRAD',
    'CWRAD',
    'CCWRAD',
    'MAXRAD',
    'MINRAD',

    'DEBRISLED',
    'SPOTLED',
    'DOCKLED',
    'CHECKLED',

    'RoombaIf',
    'RoombaError'
    ]

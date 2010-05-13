#!/usr/bin/env python
import serial
import time
import struct
import datetime
from roombaerror import *

#
# command define
#

# output command
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
SCHEDULECMD = 168
DAYCMD = 168
SONGCMD = 140
PLAYCMD = 141
DRIVEDIRECTCMD = 145

# input command
SENSORCMD = 142
QUERYCMD = 149
STREAMCMD = 148
PRSTREAMMCMD = 150


COMMAND_LIST = [STARTCMD, BAUDCMD, CONTROLCMD, SAFECMD, FULLCMD,
                POWERCMD, SPOTCMD, CLEANCMD, MAXCMD, DRIVECMD, 
                MOTORSCMD, LEDSCMD, DOCKCMD, DEGITCMD, ASCIICMD,
                SCHEDULECMD, DAYCMD, SONGCMD, PLAYCMD, DRIVEDIRECTCMD
                ]

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

#
# parameters
#
WHEEL_OFFSET = 230  #[mm]

class RoombaIf(object):
    """
    control interface for Roomba OI.
    this soft use usb serial port for controlling the robot.
    """
    def __init__(self, full=False, debug=False, device=DEFAULT_DEVICE):
        self._opened = False
        self._full = full
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
                self._ser = serial.Serial(dev,
                                          baudrate=DEFAULT_BAUD,
                                          timeout=0.1)
                self._ser.open()
            except serial.SerialException:
                raise RoombaError('serial initialize error')
            self._opened = True

    def close(self):
        if self._opened:
            self._ser.close()

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
                    self._ser.write(cmd)
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

    def _receive_data(self, num):
        if not self._debug and not self._opened:
            raise RoombaError('not opened')
        if not self._debug:
            data = self._ser.read(num)
            return data
        else:
            # debug
            data = ''
            data.zfill(num)
            return data

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

    def set_direct_vel(self, vel_r, vel_l):
        if not isinstance(vel_r, int) or not isinstance(vel_l, int):
            raise RoombaError('set vel inval error')
        if vel_r > MAXVEL:
            print ('speed limit over %d > %d' % (vel_r, MAXVEL))
            vel_r = MAXVEL
        elif vel_r < MINVEL:
            print ('speed limit over %d < %d' % (vel_r, MINVEL))
            vel_r = MINVEL

        if vel_l > MAXVEL:
            print ('speed limit over %d > %d' % (vel_l, MAXVEL))
            vel_l = MAXVEL
        elif vel_l < MINVEL:
            print ('speed limit over %d < %d' % (vel_l, MINVEL))
            vel_l = MINVEL
        try:
            cmd = struct.pack('>Bhh', DRIVEDIRECTCMD, vel_r, vel_l)
        except struct.error:
            raise RoombaError('set vel inval error')
        self._send_command(cmd)
        return (vel_r, vel_l)

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
        self._song_test()

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

    def sync_time(self):
        d = datetime.datetime.today()
        date = d.weekday() + 1
        if date == 7:
            date = 0
        try:
            bb = struct.pack('BBBB', DAYCMD, date, d.hour, d.minute)
        except struct.error:
            raise RoombaError('fatal error')
        self._send_command(bb)
        
    def set_song(self, index, song_dat):
        if index > 4 or index < 0:
            raise RoombaError('song index must be 0 - 4')
        # song_dat = ((note1, duration1), (note2, duration2), ...)
        length = len(song_dat)
        try:
            bb = struct.pack('BBB', SONGCMD, index, length)
            for note in song_dat:
                bb += struct.pack('BB', note[0], note[1])
        except struct.error:
            raise RoombaError('pack error')
        self._send_command(bb)

    def play_song(self, index):
        if index > 4 or index < 0:
            raise RoombaError('song index must be 0 - 4')
        try:
            bb = struct.pack('BB', PLAYCMD, index)
        except struct.error:
            raise RoombaError('fatal error')
        self._send_command(bb)

    def start_sensor_stream(self):
        pass

    def _get_sensor(self, sensor_id):
        try:
            bb = struct.pack('BB', SENSORCMD, sensor_id)
        except struct.error:
            raise RoombaError('fatal error')
        
        self._send_command(bb)
    
    def _get_byte_sensor(self, sensor_id):
        self._get_sensor(sensor_id)
        raw_data = self._receive_data(1)
        try:
            num_data = struct.unpack("B", raw_data)
        except struct.error:
            raise RoombaError('fatal receive data error')
        data = num_data[0]
        return data

    def _get_char_sensor(self, sensor_id):
        self._get_sensor(sensor_id)
        raw_data = self._receive_data(1)
        try:
            num_data = struct.unpack("b", raw_data)
        except struct.error:
            raise RoombaError('fatal receive data error')
        data = num_data[0]
        return data

    def _get_short_sensor(self, sensor_id):
        self._get_sensor(sensor_id)
        raw_data = self._receive_data(2)

        try:
            num_data = struct.unpack(">h", raw_data)
        except struct.error:
            raise RoombaError('fatal receive data error')
        data = num_data[0]
        return data

    def _get_ushort_sensor(self, sensor_id):
        self._get_sensor(sensor_id)
        raw_data = self._receive_data(2)
        try:
            num_data = struct.unpack(">H", raw_data)
        except struct.error:
            raise RoombaError('fatal receive data error')
        data = num_data[0]
        return data

    def get_bumper(self):
        """ return (right bumper on?, left bumper on?)
        """
        data = self._get_byte_sensor(7)
        right = ((data >> 0) & 1) == 1
        left = ((data >> 1) & 1) == 1
        return (right, left)

    def get_wheel_drop(self):
        """ return (right wheel drop?, left wheel drop?)
        """
        data = self._get_byte_sensor(7)
        right = ((data >> 2) & 1) == 1
        left = ((data >> 3) & 1) == 1
        return (right, left)
       
    def get_distance(self):
        return self._get_short_sensor(19)

    def get_angle(self):
        return self._get_short_sensor(20)

    def get_voltage(self):
        return self._get_ushort_sensor(22)

    def get_current(self):
        return self._get_short_sensor(23)

    def get_temperature(self):
        return self._get_char_sensor(24)

    def get_battery_charge(self):
        return self._get_ushort_sensor(25)


    def _song_test(self):
        self.set_song(0, ((60, 32), (64, 32), (60, 32), (64, 32)))
        self.set_song(1, ((60, 16), (64, 16), (60, 16), (64, 16)))
#        self.play_song(0)
        self.play_song(1)


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
    'RoombaError',
    'RoombaStreamError',

    'WHEEL_OFFSET'
    ]

if __name__ == '__main__':
    r = RoombaIf()
    r.setup()
#    r._song_test()
#    r.teardown()

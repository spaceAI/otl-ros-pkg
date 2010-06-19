#!/usr/bin/env python

#
# Roomba OI interface
# New BSD Liscense
#
# 2010.5.21 commented initially
# by OTL <t.ogura@gmail.com>
#

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
WHEEL_OFFSET = 225  #[mm]

class RoombaState(object):
    """Roomba sensor state object
    """
    pass

class RoombaIf(object):
    """
    control interface for Roomba OI.
    this soft use usb serial port for controlling the robot.
    """
    def __init__(self, full=False, debug=False, device=DEFAULT_DEVICE):
        """
        initialize parameters.
        this does not connect to roomba.
        please use setup to connect it after initialized.
        """
        self._opened = False
        self._full = full
        self._debug = debug
        self._device = device

    def setup(self, otl=True):
        """
        open device and initialize LED and odometry.
        and if otl=True then beep and display 'OTL' on 7-Seg LED.
        """
        self.open(self._device)
        self.activate(full=self._full)
        self.set_led(DEBRISLED, 0, 0)
        self.set_led(CHECKLED, 0, 50)
        self.set_led(DOCKLED, 0, 100)
        self.set_led(SPOTLED, 0, 150)
        self.set_led(DEBRISLED | CHECKLED | DOCKLED | SPOTLED, 0, 200)
        self.set_led(DEBRISLED | CHECKLED | DOCKLED | SPOTLED, 100, 200)
        #reset odom
        self.get_distance()
        self.get_angle()
        # 'OTL'
        if otl:
            self.set_otl()

    def teardown(self):
        """
        connection close.
        After this method, you can control nothing!
        """
        self._send_command(POWERCMD)
        # anyway close
        self.close()

    def open(self, dev=DEFAULT_DEVICE):
        """
        Open the serial port of the roomba.
        timeout is 0.1sec.
        """
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
        """close the serial port
        """
        if self._opened:
            self._ser.close()

    def _send_command(self, cmd):
        """
        send roomba command (internal use)
        """
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
        """
        receive sensor data from roomba
        """
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
        """set the roomba velocity by linear velocity and radius.
        the speed is limited by MAXVEL.
        """
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
        """set the velocity of the roomba by right wheel speed and left wheel speed.
        """
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

    # mm/s + rad/s
    def set_twist_vel(self, vel_trans, vel_rot):
        """set the roomba speed by linear vel (mm/s) and rotational vel(rad/s)
        """
        ts = vel_trans
        tw = vel_rot * (WHEEL_OFFSET / 2)
        vel_r = int(ts + tw)
        vel_l = int(ts - tw)
        self.set_direct_vel(vel_r, vel_l)

    def clean_off(self):
        """turn off the clearning motors.
        """
        try:
            cmd = struct.pack('BB', MOTORSCMD, 0)
        except:
            raise RoombaError('Fatal error in clean_off')
        self._send_command(cmd)

    def clean_on(self):
        """turn on the all clearning motors.
        """
        try:
            cmd = struct.pack('BB', MOTORSCMD, 7)
        except struct.error:
            raise RoombaError('Fatal error in clean_off')
        self._send_command(cmd)

    def set_led(self, bits, color, power):
        """set the main leds.
        you can use like below..
        self.set_led(DEBRISLED | CHECKLED | DOCKLED | SPOTLED, 0, 200)
        color and power are for main large button LED.
        color : 0 = red << green =255
        """
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
        """set the 7-seg LED by degit.
        please see the Roomba OI document for details.
        if you want to write letters, please use set_ascii method instead.
        """
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
        """OTL special method!
        """
        self.set_degit(63, 7, 1, 56)
        self._song_test()

    def set_ascii(self, ltr):
        """ you can easily set the ascii on 7-seg LED.
        you can set only 4 letters.
        """
        if isinstance(ltr, basestring) and len(ltr) == 4:
            try:
                cmd = struct.pack('>Bcccc', ASCIICMD, ltr[0], ltr[1], ltr[2], ltr[3])
            except struct.error:
                raise RoombaError('inval error in set_ascii')
        else:
            raise RoombaError('inval type error in set_ascii')
        self._send_command(cmd)

    def activate(self, full=False):
        """activate the OI interface.
        default mode is 'SAFE': if you hold up the roomba or bump to the wall, the roomba stop.
        if you want to disable the safe functions, you can use full=True.
        """
        self._send_command(STARTCMD)
        self._send_command(SAFECMD)
        if full:
            self._send_command(FULLCMD)

    def sync_time(self):
        """synclonize the clock between PC and the roomba.
        """
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
        """set the song data to index.

        after this method you can call play_song.
        """
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
        """after set_song, you can play_song.
        """
        if index > 4 or index < 0:
            raise RoombaError('song index must be 0 - 4')
        try:
            bb = struct.pack('BB', PLAYCMD, index)
        except struct.error:
            raise RoombaError('fatal error')
        self._send_command(bb)

    def start_sensor_stream(self):
        """start the sensor stream. (not implemented yet)
        please use _request_sensor method to receive data from roomba.
        """
        pass

    def _request_sensor(self, sensor_id):
        """request the data of sensor_id.
        after this method you can read the data by _receive_data
        """
        try:
            bb = struct.pack('BB', SENSORCMD, sensor_id)
        except struct.error:
            raise RoombaError('fatal error')
        
        self._send_command(bb)
    
    def _get_byte_sensor(self, sensor_id):
        self._request_sensor(sensor_id)
        raw_data = self._receive_data(1)
        try:
            num_data = struct.unpack("B", raw_data)
        except struct.error:
            raise RoombaError('fatal receive data error')
        data = num_data[0]
        return data

    def _get_char_sensor(self, sensor_id):
        self._request_sensor(sensor_id)
        raw_data = self._receive_data(1)
        try:
            num_data = struct.unpack("b", raw_data)
        except struct.error:
            raise RoombaError('fatal receive data error')
        data = num_data[0]
        return data

    def _get_short_sensor(self, sensor_id):
        self._request_sensor(sensor_id)
        raw_data = self._receive_data(2)

        try:
            num_data = struct.unpack(">h", raw_data)
        except struct.error:
            raise RoombaError('fatal receive data error')
        data = num_data[0]
        return data

    def _get_ushort_sensor(self, sensor_id):
        self._request_sensor(sensor_id)
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

    # mm
    def get_distance(self):
        """ get the odometry of translation (mm)
        
        after call this method odometry(distance) is reseted.
        """
        return -8 * self._get_short_sensor(19) # why * 8 ???

    # deg
    def get_angle(self):
        """ get the odometry of rotation (deg)
        
        after call this method odometry(angle) is reseted.
        """
        return (3 * self._get_short_sensor(20)) # why * 3 ??? for mapping
#        return (2 * self._get_short_sensor(20)) # why * 3 ???

    def get_voltage(self):
        """get the battery voltage (mV)
        """
        return self._get_ushort_sensor(22)

    def get_current(self):
        """get the battery current
        """
        return self._get_short_sensor(23)

    def get_temperature(self):
        """get the battery temperature
        """
        return self._get_char_sensor(24)

    def get_battery_charge(self):
        """get the battery charge
        """
        return self._get_ushort_sensor(25)


    def get_mode(self):
        """get the mode of OI
        """
        num = self._get_byte_sensor(35)
        if num == 0:
            mode = 'OFF'
        elif num == 1:
            mode = 'PASSIVE'
        elif num == 2:
            mode = 'SAFE'
        elif num == 3:
            mode = 'FULL'
        else:
            raise RoombaError('fatal receive data error')
        return mode

    def _song_test(self):
        """this method is used by set_otl method.

        play some sound for showing the roomba OI activated.
        """
        self.set_song(0, ((60, 32), (64, 32), (60, 32), (64, 32)))
        self.set_song(1, ((60, 8), (64, 8)))
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
    r = RoombaIf(full=True)
    r.setup()
    time.sleep(1.0)
    r.get_distance()
    r.get_angle()
    import math
    r.set_twist_vel(80, math.radians(0))
    import time
    time.sleep(1.0)
    r.set_direct_vel(0, 0)
    print r.get_distance() # mm
    print r.get_angle() # deg 

#    r._song_test()
#    r.teardown()

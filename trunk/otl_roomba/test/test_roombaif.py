#!/usr/bin/env python
import roslib
roslib.load_manifest('otl_roomba')
import sys
import unittest
from test import test_support
from otl_roomba.roombaif import *


class TestRoombaOpen(unittest.TestCase):
    def test_open_close1(self):
        self._rmb = RoombaIf()
        self._rmb.close()

    def test_open_close2(self):
        self._rmb = RoombaIf(full=True)
        self._rmb.setup()
        self._rmb.activate()
        self._rmb.teardown()
        self._rmb.close()

    def test_open_close3(self):
        self._rmb = RoombaIf(device='/dev/test')
        self.assertRaises(RoombaError, self._rmb.setup)
        self._rmb.close()

    def test_open_close32(self):
        self._rmb = RoombaIf(debug=True, device='/dev/test')
        self._rmb.setup()
        self._rmb.activate()
        self._rmb.teardown()

    def test_open_close33(self):
        self._rmb = RoombaIf(full=True)
        self._rmb.setup()
        self._rmb.teardown()

    def test_open_close34(self):
        self._rmb = RoombaIf(full=True, device='/dev/test')
        self.assertRaises(RoombaError, self._rmb.setup)

    def test_open_close35(self):
        self._rmb = RoombaIf(debug=True, full=True, device='/dev/test')
        self._rmb.setup()
        self._rmb.teardown()

    def test_open_close4(self):
        self._rmb = RoombaIf()
        self._rmb.setup(otl=False)
        self._rmb.teardown()


class TestRoombaNotOpenCommand(unittest.TestCase):
    def setUp(self):
        self._rmb = RoombaIf()

    def test_command(self):
        self.assertRaises(RoombaError, self._rmb.set_vel, 100, 100)

    def test_led(self):
        self.assertRaises(RoombaError, self._rmb.set_degit, 100, 100, 100, 100)


class TestRoombaCommand(unittest.TestCase):
    def setUp(self):
        self._rmb = RoombaIf()
        self._rmb.setup()

    def teardown(self):
        self._rmb.teardown()

    def test_command1(self):
        self.assertRaises(RoombaError, self._rmb._send_command, 999)

    def test_command2(self):
        self._rmb._send_command(128)

    def test_command3(self):
        self.assertRaises(RoombaError, self._rmb._send_command, -1)

    def test_vel1(self):
        self._rmb.set_vel(0, 0)
        self._rmb.set_vel(100, STRAIGHTRAD)
        self._rmb.set_vel(100, CWRAD)
        self._rmb.set_vel(100, CCWRAD)

    def test_vel2(self):
        self._rmb.set_vel(MAXVEL, 100)

    def test_vel3(self):
        rtn = self._rmb.set_vel(MAXVEL + 1, 100)
        self.assertEqual(rtn[0], MAXVEL)
        self.assertEqual(rtn[1], 100)

    def test_vel4(self):
        rtn = self._rmb.set_vel(100, MAXRAD)
        self.assertEqual(rtn[0], 100)
        self.assertEqual(rtn[1], MAXRAD)

    def test_vel5(self):
        rtn = self._rmb.set_vel(100, MAXRAD + 1)
        self.assertEqual(rtn[0], 100)
        self.assertEqual(rtn[1], STRAIGHTRAD)

    def test_vel6(self):
        rtn = self._rmb.set_vel(-MAXVEL, 0)
        self.assertEqual(rtn[0], -MAXVEL)
        self.assertEqual(rtn[1], 0)

    def test_vel7(self):
        rtn = self._rmb.set_vel(-MAXVEL - 1, -200)
        self.assertEqual(rtn[0], -MAXVEL)
        self.assertEqual(rtn[1], -200)

    def test_vel8(self):
        rtn = self._rmb.set_vel(0, -MAXRAD)
        self.assertEqual(rtn[0], 0)
        self.assertEqual(rtn[1], -MAXRAD)

    def test_vel9(self):
        rtn = self._rmb.set_vel(100, -MAXRAD - 1)
        self.assertEqual(rtn[0], 100)
        self.assertEqual(rtn[1], STRAIGHTRAD)

    def test_vel10(self):
        rtn = self._rmb.set_vel(-MAXVEL - 1, -MAXRAD - 1)
        self.assertEqual(rtn[0], -MAXVEL)
        self.assertEqual(rtn[1], STRAIGHTRAD)

    def test_vel11(self):
        self.assertRaises(RoombaError, self._rmb.set_vel, 'hoge', 100)

    def test_clean_off(self):
        self._rmb.clean_off()
        self._rmb.clean_off()

    def test_clean_on(self):
        self._rmb.clean_on()
        self._rmb.clean_on()

    def test_led1(self):
        self._rmb.set_led(0, 0, 0)

    def test_led2(self):
        self.assertRaises(RoombaError, self._rmb.set_led, -1, 0, 0)

    def test_led3(self):
        self.assertRaises(RoombaError, self._rmb.set_led, 0, -1, 0)

    def test_led4(self):
        self.assertRaises(RoombaError, self._rmb.set_led, 0, 0, -1)

    def test_led5(self):
        self.assertRaises(RoombaError, self._rmb.set_led, 300, 0, 0)

    def test_led6(self):
        self.assertRaises(RoombaError, self._rmb.set_led, 0, 300, 0)

    def test_led7(self):
        self.assertRaises(RoombaError, self._rmb.set_led, 0, 0, 300)

    def test_degit1(self):
        self._rmb.set_degit(1, 2, 3, 4)

    def test_degit2(self):
        self.assertRaises(RoombaError, self._rmb.set_degit, -1, 0, 0, 0)

    def test_degit3(self):
        self.assertRaises(RoombaError, self._rmb.set_degit, 0, -1, 0, 0)

    def test_degit4(self):
        self.assertRaises(RoombaError, self._rmb.set_degit, 0, 0, -1, 0)

    def test_degit5(self):
        self.assertRaises(RoombaError, self._rmb.set_degit, 0, 0, 0, -1)

    def test_degit6(self):
        self.assertRaises(RoombaError, self._rmb.set_degit, 300, 0, 0, 0)

    def test_degit7(self):
        self.assertRaises(RoombaError, self._rmb.set_degit, 0, 300, 0, 0)

    def test_degit8(self):
        self.assertRaises(RoombaError, self._rmb.set_degit, 0, 0, 300, 0)

    def test_degit9(self):
        self.assertRaises(RoombaError, self._rmb.set_degit, 0, 0, 0, 300)

    def test_otl(self):
        self._rmb.set_otl()

    def test_ascii(self):
        self._rmb.set_ascii('    ')
        self._rmb.set_ascii('1234')
        self._rmb.set_ascii('!"#(')
        self.assertRaises(RoombaError, self._rmb.set_ascii, '!"#(oijoie')

    def test_ascii2(self):
        self.assertRaises(RoombaError, self._rmb.set_ascii, 132)


if __name__ == '__main__':
    import rostest
#    test_support.run_unittest(TestRoombaOpen, TestRoombaCommand)
    rostest.unitrun('test_roombaif', 'test_roomba_open', TestRoombaOpen)
    rostest.unitrun('test_roombaif', 'test_roomba_command', TestRoombaCommand)

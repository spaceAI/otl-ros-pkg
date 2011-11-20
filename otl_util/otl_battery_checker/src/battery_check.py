#! /usr/bin/env python

import re

class AcpiChecker():
    """read /proc/acpi/battery/*** files for check the PC's battery"""
    def __init__(self, battery_name):
        self._info_path = "/proc/acpi/battery/" + battery_name + "/info"
        self._state_path = "/proc/acpi/battery/" + battery_name + "/state"
    def get_capacity(self):
        f = open(self._info_path)
        for line in f:
            match = re.search(r'^last full capacity:\s*(\d+)', line)
            if match:
                capacity = match.groups()[0]
        f.close()
        return capacity

    def get_state(self):
        f = open(self._state_path)
        for line in f:
            match = re.search(r'^remaining capacity:\s*(\d+)', line)
            if match:
                state = match.groups()[0]
        f.close()
        return state

    def get_rate(self):
        return float(self.get_state()) / float(self.get_capacity())

if __name__=='__main__':
    checker = AcpiChecker('BAT0')
    print checker.get_rate()

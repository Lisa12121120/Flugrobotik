#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2011-2013 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
# flake8: noqa
"""
Enables logging of variables from the Crazyflie.

When a Crazyflie is connected it's possible to download a TableOfContent of all
the variables that can be logged. Using this it's possible to add logging
configurations where selected variables are sent to the client at a
specified period.

#### Terminology

| Term              | Description |
| ----------------- | ----------- |
| <nobr>Log configuration</nobr> | A configuration with a period and a number of variables that are present in the TOC. |
| Stored as         | The size and type of the variable as declared in the Crazyflie firmware. |
| Fetch as          | The size and type that a variable should be fetched as. This does not have to be the same as the size and type it's stored as. |

#### States of a configuration

| State             | Description |
| ----------------- | ----------- |
| <nobr>Created on host</nobr> | When a configuration is created the contents is checked so that all the variables are present in the TOC. If not then the configuration cannot be created. |
| <nobr>Created on CF</nobr>   | When the configuration is deemed valid it is added to the Crazyflie. At this time the memory constraint is checked and the status returned. |
| <nobr>Started on CF</nobr>   | Any added block that is not started can be started. Once started the Crazyflie will send back logdata periodically according to the specified period when it's created. |
| <nobr>Stopped on CF</nobr>   | Any started configuration can be stopped. The memory taken by the configuration on the Crazyflie is NOT freed, the only effect is that the Crazyflie will stop sending logdata back to the host. |
| <nobr>Deleted on CF</nobr>   | Any block that is added can be deleted. When this is done the memory taken by the configuration is freed on the Crazyflie. The configuration will have to be re-added to be used again. |
"""
import errno
import logging
import struct


__author__ = 'Bitcraze AB'
__all__ = ['LogVariable', 'LogTocElement']

logger = logging.getLogger(__name__)


class LogVariable():
    """A logging variable"""

    TOC_TYPE = 0
    MEM_TYPE = 1

    def __init__(self, name='', fetchAs='uint8_t', varType=TOC_TYPE,
                 storedAs='', address=0):
        self.name = name
        self.fetch_as = LogTocElement.get_id_from_cstring(fetchAs)
        if (len(storedAs) == 0):
            self.stored_as = self.fetch_as
        else:
            self.stored_as = LogTocElement.get_id_from_cstring(storedAs)
        self.address = address
        self.type = varType
        self.stored_as_string = storedAs
        self.fetch_as_string = fetchAs

    def is_toc_variable(self):
        """
        Return true if the variable should be in the TOC, false if raw memory
        variable
        """
        return self.type == LogVariable.TOC_TYPE

    def get_storage_and_fetch_byte(self):
        """Return what the variable is stored as and fetched as"""
        return (self.fetch_as | (self.stored_as << 4))

    def __str__(self):
        return ('LogVariable: name=%s, store=%s, fetch=%s' %
                (self.name, LogTocElement.get_cstring_from_id(self.stored_as),
                 LogTocElement.get_cstring_from_id(self.fetch_as)))

class LogTocElement:
    """An element in the Log TOC."""
    types = {0x01: ('uint8_t', '<B', 1),
             0x02: ('uint16_t', '<H', 2),
             0x03: ('uint32_t', '<L', 4),
             0x04: ('int8_t', '<b', 1),
             0x05: ('int16_t', '<h', 2),
             0x06: ('int32_t', '<i', 4),
             0x08: ('FP16', '<e', 2),
             0x07: ('float', '<f', 4)}

    @staticmethod
    def get_id_from_cstring(name):
        """Return variable type id given the C-storage name"""
        for key in list(LogTocElement.types.keys()):
            if (LogTocElement.types[key][0] == name):
                return key
        raise KeyError('Type [%s] not found in LogTocElement.types!' % name)

    @staticmethod
    def get_cstring_from_id(ident):
        """Return the C-storage name given the variable type id"""
        try:
            return LogTocElement.types[ident][0]
        except KeyError:
            raise KeyError('Type [%d] not found in LogTocElement.types'
                           '!' % ident)

    @staticmethod
    def get_size_from_id(ident):
        """Return the size in bytes given the variable type id"""
        try:
            return LogTocElement.types[ident][2]
        except KeyError:
            raise KeyError('Type [%d] not found in LogTocElement.types'
                           '!' % ident)

    @staticmethod
    def get_unpack_string_from_id(ident):
        """Return the Python unpack string given the variable type id"""
        try:
            return LogTocElement.types[ident][1]
        except KeyError:
            raise KeyError(
                'Type [%d] not found in LogTocElement.types!' % ident)

    def __init__(self, ident=0, data=None):
        """TocElement creator. Data is the binary payload of the element."""

        self.ident = ident

        if (data):
            naming = data[1:]
            zt = bytearray((0, ))
            self.group = naming[:naming.find(zt)].decode('ISO-8859-1')
            self.name = naming[naming.find(zt) + 1:-1].decode('ISO-8859-1')

            self.ctype = LogTocElement.get_cstring_from_id(data[0])
            self.pytype = LogTocElement.get_unpack_string_from_id(data[0])

            self.access = data[0] & 0x10


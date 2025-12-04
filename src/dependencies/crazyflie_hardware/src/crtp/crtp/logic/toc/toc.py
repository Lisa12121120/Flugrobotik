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
"""
A generic TableOfContents module that is used to fetch, store and manipulate
a TOC for logging or parameters.
"""
import logging

__author__ = 'Bitcraze AB'
__all__ = ['Toc']

logger = logging.getLogger(__name__)

class Toc:
    """Container for TocElements."""

    def __init__(self):
        self.toc = {}

    def clear(self):
        """Clear the TOC"""
        self.toc = {}

    def add_element(self, element):
        """Add a new TocElement to the TOC container."""
        try:
            self.toc[element.group][element.name] = element
        except KeyError:
            self.toc[element.group] = {}
            self.toc[element.group][element.name] = element

    def get_element_by_complete_name(self, complete_name):
        """Get a TocElement element identified by complete name from the
        container."""
        try:
            return self.get_element_by_id(self.get_element_id(complete_name))
        except ValueError:
            # Item not found
            return None

    def get_element_id(self, complete_name):
        """Get the TocElement element id-number of the element with the
        supplied name."""
        [group, name] = complete_name.split('.')
        element = self.get_element(group, name)
        if element:
            return element.ident
        else:
            logger.warning('Unable to find variable [%s]', complete_name)
            return None

    def get_element(self, group, name):
        """Get a TocElement element identified by name and group from the
        container."""
        try:
            return self.toc[group][name]
        except KeyError:
            return None

    def get_element_by_id(self, ident):
        """Get a TocElement element identified by index number from the
        container."""
        for group in list(self.toc.keys()):
            for name in list(self.toc[group].keys()):
                if self.toc[group][name].ident == ident:
                    return self.toc[group][name]
        return None


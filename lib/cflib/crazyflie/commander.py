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

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Used for sending control setpoints to the Crazyflie
"""

__author__ = 'Bitcraze AB'
__all__ = ['Commander']

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from random import randint
import struct, numpy


class Commander():
    """
    Used for sending control setpoints to the Crazyflie
    """

    PACKET_ID = 0

    def __init__(self, crazyflie=None):
        """
        Initialize the commander object. By default the commander is in
        +-mode (not x-mode).
        """
        self._cf = crazyflie
        self._x_mode = False
        self.PACKET_ID = 0

    def increment_packet_id(self):
        global PACKET_ID
        if self.PACKET_ID < 65535:
            self.PACKET_ID += 1
        else:
            self.PACKET_ID = 0
        return self.PACKET_ID

    def set_client_xmode(self, enabled):
        """
        Enable/disable the client side X-mode. When enabled this recalculates
        the setpoints before sending them to the Crazyflie.
        """
        self._x_mode = enabled

    def send_setpoint(self, roll, pitch, yaw, thrust):
        """
        Send a new control setpoint for roll/pitch/yaw/thrust to the copter

        The arguments roll/pitch/yaw/trust is the new setpoints that should
        be sent to the copter
        """
        if thrust > 0xFFFF or thrust < 0:
            raise ValueError("Thrust must be between 0 and 0xFFFF")

        if self._x_mode:
            roll, pitch = 0.707 * (roll - pitch), 0.707 * (roll + pitch)

        pk = CRTPPacket()
        pk.port = CRTPPort.COMMANDER
        id = numpy.uint8(0);
        numberpack = numpy.uint16(self.increment_packet_id());
        rssi = numpy.uint8(0)
        x = numpy.uint16(0)
        y = numpy.uint16(0)
        z = numpy.uint16(0)

        pk.data = struct.pack('<BHBHHHfffH', id, numberpack, rssi, x, y, z, roll, -pitch, yaw, thrust)
        self._cf.send_packet(pk)

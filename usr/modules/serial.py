# Copyright (c) Quectel Wireless Solution, Co., Ltd.All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Author: dustin.wei
Email: dustin.wei@quectel.com
Date: 2023-02-14

Background: QuecPython's machine.UART read/write operations are non-blocking by default.
Current requirement is to implement blocking mode read operations.

Function: Encapsulate Serial class to implement blocking/non-blocking UART read operations.
"""

from machine import UART, Timer
from usr.modules.common import Condition


class TimerContext(object):
    """Timer implementation using machine.Timer (ONE_SHOT mode) with context manager support."""
    __timer = Timer(Timer.Timer1)

    def __init__(self, timeout, callback):
        """
        Args:
            timeout (int): Milliseconds. >0 starts timer, <=0 does nothing
            callback (function): Callback when timeout occurs
        """
        self.timeout = timeout
        self.timer_cb = callback

    def __enter__(self):
        if self.timeout > 0:
            self.__timer.start(period=self.timeout, mode=Timer.ONE_SHOT, callback=self.timer_cb)

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.timeout > 0:
            self.__timer.stop()


class Serial(object):
    """UART communication wrapper class"""

    def __init__(self, port=2, baudrate=115200, bytesize=8, parity=0, stopbits=1, flowctl=0):
        port = getattr(UART, 'UART{}'.format(port))
        self.__uart = UART(port, baudrate, bytesize, parity, stopbits, flowctl)
        self.__uart.set_callback(self.__uart_cb)
        self.__cond = Condition()

    def __uart_cb(self, args):
        """UART interrupt callback"""
        self.__cond.notify(info=False)

    def __timer_cb(self, args):
        """Timeout callback""" 
        self.__cond.notify(info=True)

    def write(self, data):
        """Write data to UART (non-blocking)"""
        self.__uart.write(data)

    def read(self, size, timeout=0):
        """
        Read from UART in blocking/non-blocking mode
        
        Args:
            size (int): Number of bytes to read
            timeout (int): Milliseconds. 
                =0: Non-blocking
                <0: Block indefinitely
                >0: Block with timeout
        
        Returns:
            bytes: Actual read data
        """
        if timeout == 0:
            return self.__uart.read(size)

        r_data = b''
        with TimerContext(timeout, self.__timer_cb):
            while len(r_data) < size:
                raw = self.__uart.read(1)
                if not raw:
                    if self.__cond.wait():
                        break
                else:
                    r_data += raw
        return r_data

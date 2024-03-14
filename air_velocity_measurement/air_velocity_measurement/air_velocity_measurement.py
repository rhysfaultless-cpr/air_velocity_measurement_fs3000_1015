#!/usr/bin/env python3
# Software License Agreement (BSD)
#
# @author    Rhys Faultless <rfaultless@clearpathrobotics.com>
# @copyright (c) 2024, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import serial
from threading import Lock
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from std_msgs.msg import String


class AirVelocityMeasurement(Node):
    def __init__(self):
        super().__init__('AirVelocityMeasurement')
        measurement_timer_period = 1.0  # seconds
        self.meaurement_timer = self.create_timer(measurement_timer_period, self.measurement_timer_callback)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.declare_parameter('baud', 115200)
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.serial_port = serial.Serial(self.port, self.baud, timeout=1)
        self.SERIAL_READ_SIZE = 25
        self.serial_lock = Lock()

        self.air_velocity_measurement = 0
        self.air_velocity_measurement_publisher = None


    def read_air_velocity_measurement(self):
        self.serial_lock.acquire()
        response = str(self.serial_port.read(self.SERIAL_READ_SIZE))
        self.serial_port.flush()
        self.serial_lock.release()
    
        response = response.rstrip(response[-1])
        response = response.rstrip(response[-1])
        response = response.rstrip(response[-1])
        response = response.lstrip(response[0])
        response = response.lstrip(response[0])
        response = int(float(response) * 100)
        
        self.set_air_velocity_measurement(response)


    def set_air_velocity_measurement(self, content):
        self.air_velocity_measurement = content


    def get_air_velocity_measurement(self):
        return self.air_velocity_measurement


    def update_air_velocity_measurement_publisher(self):
        self.air_velocity_measurement_publisher = self.create_publisher(Int16, 'air_velocity_measurement', 10)


    def measurement_timer_callback(self):
        measurement_msg = Int16()
        self.read_air_velocity_measurement()
        measurement_msg.data = self.get_air_velocity_measurement()
        self.air_velocity_measurement_publisher.publish(measurement_msg)


def main():
    rclpy.init()
    air_velocity_measurement = AirVelocityMeasurement()
    air_velocity_measurement.read_air_velocity_measurement()
    air_velocity_measurement.update_air_velocity_measurement_publisher()
    rclpy.spin(air_velocity_measurement)
    air_velocity_measurement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

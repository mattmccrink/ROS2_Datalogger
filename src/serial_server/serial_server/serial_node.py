import rclpy
from rclpy.node import Node
import serial
import struct
import argparse
import time

from std_msgs.msg import Int16MultiArray  # or a custom message type
from serial_interfaces.msg import Voltage, Current, RPM, Thrust, Strain, Pressure, Temperature, ADC, Encoder
from builtin_interfaces.msg import Time

## Version 1.0
# No CRC checks performed, need to add in next version

# Define the parser
parser = argparse.ArgumentParser(description='Short sample app')
# Declare an argument (`--port`), saying that the 
# corresponding value should be stored in the `algo` 
# field, and using a default value if the argument 
# isn't given
parser.add_argument('--port', action="store", dest='port', default='/dev/ttyUSB0')
parser.add_argument('--baud', action="store", dest='baud', default=115200)
parser.add_argument('--timeout', action="store", dest='timeout', default=0.5)
parser.add_argument('--rate', action="store", dest='rate', default=0.001)

args, unknown = parser.parse_known_args()
print(args)

class SerialMessage():
    def __init__(self):
       self.message_id = []
       self.bytes_2_read = []
       self.chk_sum = []
       self.time_boot_ms = []
       self.as_values = []

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')
        self.publisher_voltage = None
        self.publisher_current = None
        self.publisher_rpm = None
        self.publisher_thrust = None
        self.publisher_strain = None
        self.publisher_pressure = None
        self.publisher_temperature = None
        self.publisher_adc = None
        self.publisher_encoder = None

        self.serial_port = serial.Serial(args.port, args.baud, timeout=args.timeout)
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()	
        self.timer = self.create_timer(args.rate, self.timer_callback)

    def timer_callback(self):
        while (self.serial_port.in_waiting > 32):
          header = self.serial_port.read_until(b'>>>')[-3:]
          if (header == (b">>>")):
            ser_mes = SerialMessage()
            ser_mes.message_id = ord(self.serial_port.read(1)) #change from hex byte to number
            ser_mes.bytes_2_read = struct.unpack("<h",self.serial_port.read(2))[0] #Grab first element of tuple
            ser_mes.chk_sum = struct.unpack("BB",self.serial_port.read(2))
            ser_mes.time_boot_ms = struct.unpack("i",self.serial_port.read(4))[0]
            ser_mes.as_values = self.serial_port.read(ser_mes.bytes_2_read)
            ser_mes.as_values = struct.unpack("h"*int(ser_mes.bytes_2_read/2),ser_mes.as_values)
            message_process(self,ser_mes)
            
def message_process(self,ser_mes):

  match ser_mes.message_id:
    case 0:
        if self.publisher_voltage is None:
            self.publisher_voltage = self.create_publisher(Voltage, 'Voltage', 10)
        msg = Voltage()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.time_boot_ms = ser_mes.time_boot_ms
        msg.voltage = ser_mes.as_values  # or process as needed
        self.publisher_voltage.publish(msg)
    case 1:             
        if self.publisher_current is None:
            self.publisher_current = self.create_publisher(Current, 'Current', 10)
        msg = Current()
        msg.time_boot_ms = ser_mes.time_boot_ms
        msg.timestamp = self.get_clock().now().to_msg()
        msg.current = ser_mes.as_values  # or process as needed
        self.publisher_current.publish(msg)
    case 2:             
        if self.publisher_rpm is None:
            self.publisher_rpm = self.create_publisher(RPM, 'RPM', 10)
        msg = RPM()
        msg.time_boot_ms = ser_mes.time_boot_ms
        msg.timestamp = self.get_clock().now().to_msg()
        msg.rpm = ser_mes.as_values  # or process as needed
        self.publisher_rpm.publish(msg)
    case 3:             
        if self.publisher_thrust is None:
            self.publisher_thrust = self.create_publisher(Thrust, 'Thrust', 10)
        msg = Thrust()
        msg.time_boot_ms = ser_mes.time_boot_ms
        msg.timestamp = self.get_clock().now().to_msg()
        msg.thrust = ser_mes.as_values  # or process as needed
        self.publisher_thrust.publish(msg)
    case 4:             
        if self.publisher_strain is None:
            self.publisher_strain = self.create_publisher(Strain, 'Strain', 10)
        msg = Strain()
        msg.time_boot_ms = ser_mes.time_boot_ms
        msg.timestamp = self.get_clock().now().to_msg()
        msg.strain = ser_mes.as_values  # or process as needed
        self.publisher_strain.publish(msg)
    case 5:             
        if self.publisher_pressure is None:
            self.publisher_pressure = self.create_publisher(Pressure, 'Pressure', 10)
            char_to_send = "Z"
            self.serial_port.write(char_to_send.encode())
        msg = Pressure()
        msg.time_boot_ms = ser_mes.time_boot_ms
        msg.timestamp = self.get_clock().now().to_msg()
        msg.pressure = ser_mes.as_values  # or process as needed
        self.publisher_pressure.publish(msg)
    case 6:             
        if self.publisher_temperature is None:
            self.publisher_temperature = self.create_publisher(Temperature, 'Temperature', 10)
        msg = Temperature()
        msg.time_boot_ms = ser_mes.time_boot_ms
        msg.timestamp = self.get_clock().now().to_msg()
        msg.temperature = ser_mes.as_values  # or process as needed
        self.publisher_temperature.publish(msg)
    case 7:             
        if self.publisher_adc is None:
            self.publisher_adc = self.create_publisher(ADC, 'ADC', 10)
        msg = ADC()
        msg.time_boot_ms = ser_mes.time_boot_ms
        msg.timestamp = self.get_clock().now().to_msg()
        msg.adc = ser_mes.as_values  # or process as needed
        self.publisher_adc.publish(msg)
    case 8:             
        if self.publisher_encoder is None:
            self.publisher_encoder = self.create_publisher(Encoder, 'Encoder', 10)
        msg = Encoder()
        msg.time_boot_ms = ser_mes.time_boot_ms
        msg.timestamp = self.get_clock().now().to_msg()
        msg.encoder = ser_mes.as_values  # or process as needed
        self.publisher_encoder.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    rclpy.spin(serial_node)
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

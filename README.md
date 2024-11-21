Node supporting serial interface to low-power periperals (Arduino, etc.) not supporting uXRCS. Messages can be added in serial_server.py based on prototypes in serial_interface/msg.

To build (1st time):
1) cd to cloned directory
2) colcon build
3) echo "source install/local_setup.bash" >> ~/.bashrc

Header comprised of 4 bytes:
1) "!A"
2) msg_ID (see serial_server.py for definitions)
3) length (limit 255 bytes)

Payload interpreted as signed words (int16) with length = number of words * 2 (total bytes transfered)
No error checking is performed to maximize transfer speed

Run ROS node as follows:
ros2 run serial_server serial_node &

Install script for ROS2 Humble on Ubuntu 22.04 included in src directory. Likely to change as packages mature, so take with a grain of salt.

folders in this folder are for jetson TX2 catkin_ws/src

rules files must be in /etc/udev/rules.d

$ sudo groupadd gpio

$ sudo usermod -a -G gpio <username>

IMU serial communication
$ cat < /dev/ttyUSB0 && echo 'data' > /dev/ttyUSB0

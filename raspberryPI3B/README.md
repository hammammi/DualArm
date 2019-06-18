Epos_Linux_Library -> home 또는 desktop에 다운로드

$ sudo bash install.sh



catkin_make 시 /opt permission denied error occurs:

$ sudo chown -R $USER /opt

# ROS_MobilePlatform


- Install slcan module to Raspberry Pi </br>
https://wiki.linklayer.com/index.php/CANtact_on_Raspberry_Pi

-- > 따라서 진행해보았으나 ubuntu 18.04가 사용하는 kernel에서는 run rpi-source 부분이 진행되지 않음.

따라서 해당 사항을 건너뛰었음.

$ sudo apt-get install can-utils

사용할 때마다 terminal 창에 써야 하는듯

$ sudo modprobe can
$sudo modprobe slcan


- Raspberry Pi CAN communication

$ sudo slcand -o -c -f -s4 /dev/ttyUSB0 slcan0

$ sudo ifconfig slcan0 up

- CAN Communication monitor

$ candump slcan0




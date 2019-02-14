# ROS_MobilePlatform

- Raspberry Pi CAN communication

sudo slcand -o -c -f -s4 /dev/ttyUSB0 slcan0

sudo ifconfig slcan0 up

- CAN Communication monitor

candump slcan0


---------------------------------------------------------------------
- Install slcan module to Raspberry Pi </br>
https://wiki.linklayer.com/index.php/CANtact_on_Raspberry_Pi

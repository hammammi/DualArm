Epos_Linux_Library -> home 또는 desktop에 다운로드

$ sudo bash install.sh



catkin_make 시 /opt permission denied error occurs:

$ sudo chown -R $USER /opt

# ROS_MobilePlatform


- Install slcan module to Raspberry Pi </br>
https://wiki.linklayer.com/index.php/CANtact_on_Raspberry_Pi

-- > 따라서 진행해보았으나 ubuntu 18.04가 사용하는 kernel에서는 run rpi-source 부분이 진행되지 않음.
-- > 그냥 ubuntu 는 raspberry 상에서 너무 느려 사용 불가능, topic만 주고 받는 용도로 사용되기 때문에 이전 버전인 16.04 버전을 사용토록

따라서 해당 사항을 건너뛰었음.

$ sudo apt-get install can-utils



$ sudo modprobe can
$ sudo modprobe slcan

// CANUSB
- CAN communication

$ sudo slcand -o -c -f -s8 /dev/ttyUSBx slcan0  

$ sudo ifconfig slcan0 up  

- CAN Communication monitor  

$ candump slcan0  

- CANUSB serial port latency set up  

$ sudo apt-get install setserial  

$ sudo setserial /dev/ttyUSBx low_latency  

// PiCAN2 (user_guide 참고)  
- Add overlays  

$ sudo nano /boot/config.txt  

Add  

dtparam=spi=on  
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25  
dtoverlay=spi-bcm2835-overlay  

- Bring up interface  

$ sudo /sbin/ip link set can0 up type can bitrate 1000000  

https://www.raspberrypi.org/forums/viewtopic.php?t=190868
http://youness.net/raspberry-pi/raspberry-pi-can-bus
https://github.com/raspberrypi/linux/issues/2767

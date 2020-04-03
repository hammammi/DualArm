Raspberry Pi 3b Ubuntu 16.04

Epos_Linux_Library -> home 또는 desktop에 다운로드

$ sudo bash install.sh



catkin_make 시 /opt permission denied error occurs:

$ sudo chown -R $USER /opt

# ROS_MobilePlatform


- Install slcan module to Raspberry Pi </br>
https://wiki.linklayer.com/index.php/CANtact_on_Raspberry_Pi

참고 : http://pascal-walter.blogspot.com/2015/08/installing-lawicel-canusb-on-linux.html

$ sudo apt-get install can-utils
$ sudo modprobe can   
$ sudo modprobe can_raw   
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

Add at SPI 부분 

dtparam=spi=on  
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25  
dtoverlay=spi-bcm2835-overlay  

$ sudo reboot   

- Bring up interface  

$ sudo /sbin/ip link set can0 up type can bitrate 1000000  

if you want to make auto-start CAN interface on bootup

in /etc/network/interfaces, copy and paste  

````
auto can0  
iface can0 inet manual  
    pre-up ip link set $IFACE type can bitrate 1000000 listen-only off  
    up /sbin/ifconfig $IFACE up  
    down /sbin/ifconfig $IFACE down  
````
   
참고 : http://www.embeddedpi.com/documentation/isolated-canbus/mypi-industrial-raspberry-pi-can-bus-card-configuration    

// xenomai on raspberryPI3   

boot image 만들기 (kernel build)   

참고 : https://lemariva.com/blog/2018/07/raspberry-pi-xenomai-patching-tutorial-for-kernel-4-14-y
 
error 
-->  libz.so.1: cannot open shared object file: No such file or directory   
$ sudo apt-get install lib32z1  



https://github.com/lemariva/RT-Tools-RPi/tree/master/xenomai/v3.0.7    

사용   
in rpi   
xenomai-kernel.tgz 


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

참고 : https://github.com/thanhtam-h/rpi23-4.9.80-xeno3/tree/master/prebuilt   
 
error 
--> No ruls to make target 'arch/arm/tools/gen-mach-types' 또는 'arch/arm/tools/mach-types'   
--> fatal error: tools/be_byteshift.h: No such file or directory   
 : 위 참고 github에서 파일을 직접 다운받아 위치에 넣는 형식으로 해결     
따라서 그냥 rpi23-4.9.80-xeno3 의 master branch를 다운받은 다음 직접 deploy 하는 것이 덜 귀찮을 것이다.
 

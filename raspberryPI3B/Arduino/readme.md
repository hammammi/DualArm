arduino code for sensor reading     

msg 폴더 내의 msg 파일 : vehicle_control 패키지에 복사 후 cm   

* arduino due를 사용하였음
* due를 이용하기 위해 IDE에서 드라이버 설치 필요, board는 due programming port 

arduino 설치     

$ sudo apt-add-repository ppa:ubuntu-desktop/ubuntu-make   
$ sudo apt-get update   
$ sudo apt-get install ubuntu-make   

$ sudo umake ide arduino
$ sudo usermod -aG dialout $USER    

로그아웃 후 로그인   

IDE에서 manage libraries를 이용해 pololu의 VL53L1X 라이브러리 설치   
Arduino 폴더에 mobileplatform_sensor 폴더 복사   

rosserial_arduino 패키지 설치    

$ sudo apt-get install ros-melodic-rosserial-arduino    
$ sudo apt-get install ros-melodic-rosserial    
  
  ros_lib library 설치 (custom msg가 있을 경우 cm 후 설치)     
$ cd /libraries --> arduino 폴더 설치된 곳의 libraries 폴더 안을 말함.   
$ rm -rf ros_lib --> 기존에 ros_lib이 설치되어 있었던 경우만   
$ rosrun rosserial_arduino make_libraries.py . --> 마지막에 .을 찍어야함 (현재폴더에 설치함을 뜻함)   

mobileplatform_sensor.ino 파일 업로드

ros 노드 실행

$ roscore   
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 (arduino가 연결된 포트 적기)    



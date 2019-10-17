**for jetson Xavier AXG

rules files must be in /etc/udev/rules.d

-- for realsense

참고  https://github.com/jetsonhacks/buildLibrealsense2Xavier

위 사이트를 참고하기 위해 L4T 32.1 버전이 필요하여 Jetpack SDK 사용시 4.2 버전으로 flash해야한다.

T265를 사용하기 위해
 ./installLibrealsense.sh 실행 전 cmake 부분에 -DBUILD_WITH_TM2=true 추가
 LIBREALSENSE_VERSION=v2.25.0. 으로 변경 (추후 최신버전으로 변경해야 할지는 모르겠음)
 --> librealsesnse2가 설치되며 사용된다.
 
 Kernel은 제대로 업데이트 된건지는 잘모르겠다. uname -r 에서 변화는 없었다
 
 참고 https://github.com/IntelRealSense/realsense-ros
 
 realsense-ros 사용 시 dependency package를 우선적으로 설치 필요, > travis.yml 파일 참고.
 ros melodic version이 아직 공식적으로 나오지 않아 주의. ubuntu 18.04, melodic에 설치 시 kinetic->melodic으로 변경하여 설치
 
 failed to load nodelet 에러가 나타나는 경우 rosnode kill 이용

-- for lidar (wiki 참고)

$ sudo apt-get install ros-melodic-velodyne

dual_VLP16_points.launch >> /opt/ros/melodic/share/velodyne_pointcloud/launch

connect lidars
(이전에 각 lidar의 ip 변경 필요)

disconnect Wifi

$ sudo ifconfig eth1 192.168.3.70
$ sudo route add 192.168.1.201 eth1    (lidar ip)

$ sudo ifconfig eth2 192.168.3.71
$ sudo route add 192.168.1.202 eth2

test

$ roslaunch velodyne_pointcloud dual_VLP16_points.launch

rostopic list와 echo로 topic이 잘 나오는지 확인


fixde frame으로 rviz를 통한 확인

$ rosrun rviz rviz -f vlp1
displays에서 add-pointcloud2 vlp1/velodyne_points

아직 frame 및 TF가 없어서 동시에 보진 못함




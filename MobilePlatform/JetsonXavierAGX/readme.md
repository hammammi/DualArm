**for jetson Xavier AXG

rules files must be in /etc/udev/rules.d

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

아직 frame을 및 TF가 없어서 동시에 보진 못함

-- for realsense

참고  https://github.com/jetsonhacks/buildLibrealsense2Xavier

T265를 사용하기 위해
 ./installLibrealsense.sh 실행 전 cmake 부분에 -DBUILD_WITH_TM2=true 추가
 
 Kernel은 제대로 업데이트 된건지는 잘모르겠다. uname -r 에서 변화는 없었다


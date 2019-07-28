**for jetson Xavier AXG

rules files must be in /etc/udev/rules.d

-- for lidar (wiki 참고)

$ sudo apt-get install ros-melodic-velodyne

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


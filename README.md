# zumo32u4
**SLAM by RaspberryPi3 + Zumo32u4 + RPLidarA1M8**<br><br>
【Japanese article **Gmapping**】 https://qiita.com/PINTO/items/9aa737c284dc4e8212f1<br><br>
【Japanese article **Google CartoGrapher-01**】 https://qiita.com/PINTO/items/4845c438cac05eda4d1e<br>
【Japanese article **Google CartoGrapher-02**】 https://qiita.com/PINTO/items/fb0d44b2bb9455800667<br>
【Japanese article **Google CartoGrapher-03**】 https://qiita.com/PINTO/items/d8fd7a91ee00df7702b7<br>
【Japanese article **Google CartoGrapher-04**】 https://qiita.com/PINTO/items/f8fa5d6ce55317ea590b<br><br>
**＜Gmapping＞**<br>
![MappingView](https://github.com/PINTO0309/zumo32u4/blob/master/media/127.png)<br><br>
**＜Google CartoGrapher＞**<br>
No odometry and no IMU<br>
Use 2D LiDAR only<br>
![CartoGrapherMapMov](https://github.com/PINTO0309/zumo32u4/blob/master/media/GoogleCartoGrapherNoneOdom%2BIMU_Movie.gif)<br>
![CartoGrapherMap](https://github.com/PINTO0309/zumo32u4/blob/master/media/GoogleCartoGrapherNoneOdom%2BIMU_Map.png)<br>
With odometry and IMU and 2D LiDAR<br>
![CartoGrapherMapwodomimu](https://github.com/PINTO0309/zumo32u4/blob/master/media/GoogleCartoGrapherWithOdom%2BIMU.gif)<br>

## ◆ Change log<br>
2018.05.05 Ver 0.1.0 Under development<br>
2018.05.12 Ver 1.0.0 First Release<br>
2018.06.10 Ver 1.0.1 Bug fix<br>

## ◆ Environment<br>
![Environment](https://github.com/PINTO0309/zumo32u4/blob/master/media/0021_GMapping_English.png)
1. Zumo32u4 (ATmega32u4)
2. RaspberryPi3 (Raspbian Stretch + ROS kinetic)
3. RPLidar A1M8 (SDK ver 1.5.7 / Firmware ver 1.20)
4. Python2.7
5. C++
6. ROS kinetic
7. Working PC1 (Ubuntu16.04 + ROS kinetic + ArduinoIDE 1.8.5)
8. Working PC2 (Windows10 + TeraTerm)
<br><br>![SWStack](https://github.com/PINTO0309/zumo32u4/blob/master/media/0023_SWStack_RaspberryPi_English.png)

## ◆ Procedure for environment building

### **Perform work with Ubuntu16.04**<br>

1. Execute below
```
$ export ROS_MASTER_URI=http://raspberrypi.local:11311/

OR

$ export ROS_MASTER_URI=http://<RaspberryPi IPaddress>:11311/
```

### **Perform work with RaspberryPi3**<br>

2. Introduction of navigation package. Execute below.
```
$ sudo apt update;sudo apt upgrade
$ sudo apt install -y libbullet-dev libsdl-dev libsdl2-2.0 \
libsdl2-dev libsdl-image1.2-dev libyaml-cpp0.3-dev libyaml-cpp0.3v5

$ cd ~
$ sudo apt install cmake-curses-gui
$ git clone -b debian/kinetic/bfl https://github.com/PINTO0309/bfl-release.git
$ cd bfl-release;mkdir build;cd build
$ ccmake ..
```
3. Press the "c" key
4. Wait about 30 seconds
5. Press the "e" key
6. Set as shown below
![ccmake](https://github.com/PINTO0309/zumo32u4/blob/master/media/ccmake.png)
7. Introduction of navigation package. Execute below.
```
$ make -j3
$ sudo make install

$ cd ~
$ sudo apt install -y liborocos-kdl-dev liborocos-kdl1.3 libflann1.9 libflann-dev \
libpcl-apps1.8 libpcl-common1.8 libpcl-dev libpcl-doc libpcl-features1.8 \
libpcl-filters1.8 libpcl-io1.8 libpcl-kdtree1.8 libpcl-keypoints1.8 libpcl-ml1.8 \
libpcl-octree1.8 libpcl-outofcore1.8 libpcl-people1.8 libpcl-recognition1.8 \
libpcl-registration1.8 libpcl-sample-consensus1.8 libpcl-search1.8 \
libpcl-segmentation1.8 libpcl-stereo1.8 libpcl-surface1.8 libpcl-tracking1.8 \
libpcl-visualization1.8 pcl-tools libvtk6-dev libvtk6-java libvtk6-jni \
libvtk6-qt-dev libvtk6.3 libvtk6.3-qt python-vtk6 tcl-vtk6 vtk6 vtk6-doc \
vtk6-examples libpoco-dev libpococrypto46 libpocodata46 libpocodatamysql46 \
libpocodataodbc46 libpocodatasqlite46 libpocofoundation46 libpocomongodb46 \
libpoconet46 libpoconetssl46 libpocoutil46 libpocoxml46 libpocozip46

$ cd /
$ sudo wget https://github.com/PINTO0309/uEye_4.90.0_Linux_Arm64hf/raw/master/uEye_4.90.0_Linux_Arm64hf.tgz
$ sudo tar -xzvf uEye_4.90.0_Linux_Arm64hf.tgz
$ sudo rm uEye_4.90.0_Linux_Arm64hf.tgz
$ sudo /usr/local/share/ueye/bin/ueyesdk-setup.sh

$ sudo nano /etc/dphys-swapfile
CONF_SWAPSIZE=2048
$ sudo /etc/init.d/dphys-swapfile restart swapon -s

$ export CMAKE_PREFIX_PATH=/opt/ros/kinetic:/home/pi/catkin_ws/devel:/home/pi/catkin_ws/install
$ cd ~/catkin_ws/src
$ git clone -b kinetic-devel https://github.com/PINTO0309/geometry2.git; \
git clone https://github.com/PINTO0309/dynamic_reconfigure.git; \
git clone https://github.com/PINTO0309/laser_geometry.git; \
git clone https://github.com/PINTO0309/navigation_msgs.git; \
git clone https://github.com/PINTO0309/perception_pcl.git; \
git clone https://github.com/PINTO0309/pcl_msgs.git; \
git clone https://github.com/PINTO0309/nodelet_core.git; \
git clone https://github.com/PINTO0309/bond_core.git; \
git clone -b kinetic-devel https://github.com/PINTO0309/pluginlib.git; \
git clone https://github.com/PINTO0309/class_loader.git; \
git clone https://github.com/PINTO0309/image_common.git; \
git clone https://github.com/PINTO0309/vision_opencv.git; \
git clone https://github.com/PINTO0309/geometry.git; \
git clone https://github.com/PINTO0309/roslint.git; \
git clone https://github.com/PINTO0309/navigation.git

$ cd ..
$ catkin_make -j1
$ sudo reboot
```
8. Introduction of openslam_gmapping package and slam_gmapping meta package. Execute below.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/PINTO0309/openslam_gmapping.git
$ git clone https://github.com/PINTO0309/slam_gmapping.git

$ cd ..
$ catkin_make -j1
```
9. Introduction of RPLidarROS package. Execute below.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/PINTO0309/rplidar_ros.git
$ cd ..
$ catkin_make
```
### **Perform work with Ubuntu16.04**<br>

10. Introduction of rviz option package. Execute below.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/PINTO0309/visualization_tutorials.git
$ git clone https://github.com/PINTO0309/joint_state_publisher.git
$ git clone https://github.com/PINTO0309/robot_state_publisher.git
$ catkin_make
```
11. Remodel the rosserial library. Execute below.
```
$ cd /home/<username>/Arduino/libraries/Rosserial_Arduino_Library/src
$ cp ros.h BK_ros.h
$ sudo nano ros.h

#else
  typedef NodeHandle_<ArduinoHardware, 15, 15, 128, 1024, FlashReadOutBuffer_> NodeHandle;

$ cd ~/catkin_ws/src
$ git clone https://github.com/PINTO0309/zumo32u4.git
```
12. After connecting Ubuntu and Arduino via a microUSB cable, open zumo32u4/zumo32u4arduino.ino in ArduinoIDE in the set of programs cloned above.
13. Write a sketch to Arduino with "→" button of ArduinoIDE.

### **Perform work with RaspberryPi3**<br>

14. Execute below.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/PINTO0309/zumo32u4.git
$ cd ..
$ catkin_make
```

## ◆ Execution and visualization of map generation by RaspberryPi + Arduino + RPLidar A1M8<br>

### **Perform work with RaspberryPi3**<br>

**Terminal-1**
```
$ roscore
```
**Terminal-2**
```
$ rosrun zumo32u4 zumo32u4.py
```
**Terminal-3**
```
$ roslaunch rplidar_ros rplidar.launch
```
**Terminal-4**
```
$ roslaunch rplidar_ros slam.launch
```
**Terminal-5**
```
$ rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link laser 100
$ rosrun tf static_transform_publisher 0 0 0 0 0 0 odom base_link 100
```

### **Perform work with Ubuntu**<br>

**Terminal-1**
```
$ export ROS_MASTER_URI=http://raspberrypi.local:11311/
OR
$ export ROS_MASTER_URI=http://<RaspberryPi IPaddress>:11311/

$ roscd zumo32u4
$ roslaunch zumo32u4 zumo32u4rviz.launch
```

## ◆ Zumo32u4 operation key mapping<br>

Connect from Windows 10 PC to Raspberry Pi with TeraTerm.<br>
And, you can control by the following key operation.
![MotorControl](https://github.com/PINTO0309/zumo32u4/blob/master/media/0022_zumo32u4control.png)

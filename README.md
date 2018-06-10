# zumo32u4 (Assembled with 100:1 HP Motors)
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
With odometry and IMU and 2D LiDAR<br>
![CartoGrapherMapwodomimu](https://github.com/PINTO0309/zumo32u4/blob/master/media/GoogleCartoGrapherWithOdom%2BIMU.gif)<br>
![CartoGrapherMap](https://github.com/PINTO0309/zumo32u4/blob/master/media/GoogleCartoGrapherNoneOdom%2BIMU_Map.png)<br>

## ◆ Change log<br>
2018.05.05 Ver 0.1.0 Under development<br>
2018.05.12 Ver 1.0.0 First Release<br>
2018.06.10 Ver 1.0.1 Bug fix. Maintenance continues for Readme.<br>

## ◆ Environment<br>
![Environment](https://github.com/PINTO0309/zumo32u4/blob/master/media/0021_GMapping_English.png)
![SWStack](https://github.com/PINTO0309/zumo32u4/blob/master/media/0023_SWStack_RaspberryPi_English.png)
![Robot01](https://github.com/PINTO0309/zumo32u4/blob/master/media/150.png)
![Robot02](https://github.com/PINTO0309/zumo32u4/blob/master/media/151.png)
![Robot03](https://github.com/PINTO0309/zumo32u4/blob/master/media/152.png)
1. Zumo32u4 (ATmega32u4 Assembled with 100:1 HP Motors)
2. RaspberryPi3 (Raspbian Stretch + ROS kinetic)
3. RPLidar A1M8 (SDK ver 1.5.7 / Firmware ver 1.20)
4. Python2.7
5. C++
6. ROS kinetic
7. Working PC1 (Ubuntu16.04 + ROS kinetic + ArduinoIDE 1.8.5)
8. Working PC2 (Windows10 + TeraTerm)<br><br>

## ◆ Procedure for environment building

### **Perform work with Ubuntu16.04**<br>

1. Execute below
```
$ export ROS_MASTER_URI=http://raspberrypi.local:11311/

OR

$ export ROS_MASTER_URI=http://<RaspberryPi IPaddress>:11311/
```

### **Perform work with RaspberryPi3 (Raspbian Stretch)**<br>

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

### **Perform work with RaspberryPi3 (Raspbian Stretch)**<br>

14. Execute below.
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/PINTO0309/zumo32u4.git
$ cd ..
$ catkin_make
```

## ◆ Execution and visualization of map generation by RaspberryPi + Arduino + RPLidar A1M8<br>

### **Perform work with RaspberryPi3 (Raspbian Stretch)**<br>

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
<br><br><br><hr>
## ◆ Introduction of Google CartoGrapher<br>

### **Perform work with Ubuntu16.04**<br>

1. Execute below.
```
$ sudo apt update
$ sudo apt install -y python-wstool python-rosdep ninja-build
$ cd catkin_ws
$ wstool init src
$ wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
$ wstool update -t src
$ src/cartographer/scripts/install_proto3.sh
$ sudo rosdep init   #<--- エラーになっても無視
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
$ catkin_make_isolated --install --use-ninja
$ source install_isolated/setup.bash
```

### **Perform work with RaspberryPi3 (Raspbian Stretch)**<br>

2. Execute below.
```
$ sudo apt update;sudo apt upgrade
$ sudo apt install -y python-wstool python-rosdep ninja-build
$ cd catkin_ws
$ wstool init src
$ wstool merge -t src https://raw.githubusercontent.com/googlecartographer/cartographer_ros/master/cartographer_ros.rosinstall
$ wstool update -t src
$ src/cartographer/scripts/install_proto3.sh
$ sudo rosdep init   #<--- Ignoring an error
$ rosdep update
$ rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```
3. Execute below.
```
$ sudo nano /etc/dphys-swapfile
CONF_SWAPSIZE=2048
$ sudo /etc/init.d/dphys-swapfile restart swapon -s
$ sudo apt install libsuitesparse-dev libsuitesparseconfig4 libcxsparse3 \
libmetis-dev libmetis5 libmetis5-dbg metis libtbb-dev libtbb2 lua-bit32 \
lua-bit32-dev liblua5.3-0 liblua5.3-0-dbg liblua5.3-dev lua5.3 sphinx-common \
liburdfdom-tools libogre-1.9.0v5 ogre-1.9-tools libogre-1.9-dev \
assimp-utils libassimp3v5 python-pyassimp libassimp-dev \
qt3d-assimpsceneio-plugin sip-dev librviz-dev librviz2d python-rviz rviz
$ cd src
$ git clone https://github.com/PINTO0309/urdf.git
$ git clone https://github.com/PINTO0309/urdfdom_headers.git
$ git clone https://github.com/PINTO0309/rosconsole_bridge.git
$ cd urdfdom_headers
$ mkdir build && cd build && cmake ../ && make && sudo make install
$ cd ~/catkin_ws/src
$ git clone https://github.com/PINTO0309/urdfdom.git
$ cd urdfdom
$ mkdir build && cd build && cmake ../ && make && sudo make install
$ cd ~/catkin_ws/src
$ git clone https://github.com/PINTO0309/interactive_markers.git
$ git clone https://github.com/PINTO0309/python_qt_binding.git
$ git clone https://github.com/PINTO0309/resource_retriever.git
$ git clone https://github.com/PINTO0309/robot_state_publisher.git
$ git clone https://github.com/PINTO0309/kdl_parser.git
$ git clone https://github.com/PINTO0309/rplidar_ros.git
$ git clone https://github.com/PINTO0309/rosserial.git
$ git clone https://github.com/PINTO0309/zumo32u4.git
$ cd ~
$ git clone https://github.com/PINTO0309/pybindgen.git
$ cd pybindgen
$ sudo python setup.py install
$ cd ~/catkin_ws/src/cartographer_ros
$ rm -r -f cartographer_rviz
$ cd ~/catkin_ws
$ catkin_make_isolated -j1 --install --use-ninja
$ source install_isolated/setup.bash
```

### **Perform work with Ubuntu16.04**<br>
4. Execute below.
```
$ cd ~/catkin_ws/src/zumo32u4/urdf
$ cp zumo32u4.STL ~/catkin_ws/install_isolated/share/cartographer_ros/urdf
$ cp zumo32u4.gv ~/catkin_ws/install_isolated/share/cartographer_ros/urdf
$ cp zumo32u4.urdf ~/catkin_ws/install_isolated/share/cartographer_ros/urdf
$ nano ~/catkin_ws/install_isolated/share/cartographer_ros/urdf/zumo32u4.urdf
```
```
<geometry>
    <mesh filename="package://cartographer_ros/urdf/zumo32u4.STL" scale="1 1 1"/>
</geometry>
```
5. Execute below.
```
$ nano ~/catkin_wsgc/src/cartographer_ros/cartographer_ros/launch/backpack_2d.launch
```
```
<?xml version="1.0" ?>
<launch>
  <param
      name="robot_description"
      textfile="$(find cartographer_ros)/urdf/zumo32u4.urdf"
  />
  <node
      name="rviz"
      pkg="rviz"
      type="rviz"
      required="true"
      args="-d $(find cartographer_ros)/configuration_files/demo_2d.rviz"
  />
</launch>
```
6. Execute below.
```
$ nano ~/catkin_wsgc/src/cartographer_ros/cartographer_ros/configuration_files
```
```
Panels:
  - Class: rviz/Displays
    Help Height: 0
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Submaps1
        - /PointCloud21
        - /Map1
      Splitter Ratio: 0.600670993
    Tree Height: 322
  - Class: rviz/Selection
    Name: Selection
  - Class: rviz/Tool Properties
    Expanded:
      - /2D Pose Estimate1
      - /2D Nav Goal1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.588679016
  - Class: rviz/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: PointCloud2
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.0299999993
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
        base_link:
          Value: true
        horizontal_laser_link:
          Value: true
        map:
          Value: true
        odom:
          Value: true
      Marker Scale: 1
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        map:
          base_link:
            horizontal_laser_link:
              {}
            odom:
              {}
      Update Interval: 0
      Value: true
    - Alpha: 1
      Class: rviz/RobotModel
      Collision Enabled: false
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
        base_link:
          Alpha: 1
          Show Axes: false
          Show Trail: false
          Value: true
        map:
          Alpha: 1
          Show Axes: false
          Show Trail: false
        odom:
          Alpha: 1
          Show Axes: false
          Show Trail: false
      Name: RobotModel
      Robot Description: robot_description
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: Submaps
      Enabled: true
      Fade-out distance: 1
      High Resolution: true
      Low Resolution: false
      Name: Submaps
      Submap query service: /submap_query
      Submaps:
        All: true
        Trajectory 0:
          0.43: true
          Value: true
      Topic: /submap_list
      Tracking frame: base_link
      Unreliable: false
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz/PointCloud2
      Color: 0; 255; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 4096
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: PointCloud2
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.0500000007
      Style: Flat Squares
      Topic: /scan_matched_points2
      Unreliable: false
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Class: rviz/MarkerArray
      Enabled: true
      Marker Topic: /trajectory_node_list
      Name: Trajectories
      Namespaces:
        Trajectory 0: true
      Queue Size: 100
      Value: true
    - Class: rviz/MarkerArray
      Enabled: true
      Marker Topic: /landmark_poses_list
      Name: Landmark Poses
      Namespaces:
        {}
      Queue Size: 100
      Value: true
    - Class: rviz/MarkerArray
      Enabled: true
      Marker Topic: /constraint_list
      Name: Constraints
      Namespaces:
        Inter constraints, different trajectories: true
        Inter constraints, same trajectory: true
        Inter residuals, different trajectories: true
        Inter residuals, same trajectory: true
        Intra constraints: true
        Intra residuals: true
      Queue Size: 100
      Value: true
    - Alpha: 0.699999988
      Class: rviz/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic: /map
      Unreliable: false
      Use Timestamp: false
      Value: true
    - Class: rviz/Image
      Enabled: true
      Image Topic: /camera/image_raw
      Max Value: 1
      Median window: 5
      Min Value: 0
      Name: Image
      Normalize Range: true
      Queue Size: 2
      Transport Hint: raw
      Unreliable: false
      Value: true
  Enabled: true
  Global Options:
    Background Color: 100; 100; 100
    Default Light: true
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
    - Class: rviz/SetInitialPose
      Topic: /initialpose
    - Class: rviz/SetGoal
      Topic: /move_base_simple/goal
    - Class: rviz/PublishPoint
      Single click: true
      Topic: /clicked_point
  Value: true
  Views:
    Current:
      Angle: 0
      Class: rviz/TopDownOrtho
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.0599999987
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.00999999978
      Scale: 92.3554459
      Target Frame: <Fixed Frame>
      Value: TopDownOrtho (rviz)
      X: 0.837544441
      Y: 1.4433645
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 848
  Hide Left Dock: false
  Hide Right Dock: true
  Image:
    collapsed: false
  QMainWindow State: 000000ff00000000fd0000000400000000000001c5000002c6fc0200000009fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000006100fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000002800000183000000d700fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb0000000a0049006d00610067006501000001b10000013d0000001600ffffff000000010000010f000002e2fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a005600690065007700730000000028000002e2000000ad00fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000005ff0000003efc0100000002fb0000000800540069006d00650100000000000005ff0000030000fffffffb0000000800540069006d0065010000000000000450000000000000000000000434000002c600000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: true
  Width: 1535
  X: 55
  Y: 14
```
7. Execute below.
```
$ cd ~/catkin_ws
$ catkin_make_isolated --install --use-ninja
$ source install_isolated/setup.bash
```

### **Perform work with RaspberryPi3 (Raspbian Stretch)**<br>
8. Execute below.
```
$ cd ~/catkin_ws/src/cartographer_ros/cartographer_ros/launch
$ cp backpack_2d.launch BK_backpack_2d.launch
$ nano backpack_2d.launch
```
```
<?xml version="1.0" ?>
<launch>
  <param
      name="robot_description"
      textfile="$(find zumo32u4)/urdf/zumo32u4.urdf"
  />
  <node
     name="horizontal_laser"
     pkg="rplidar_ros"
     type="rplidarNode"
     output="screen">
     <param name="serial_port"      type="string" value="/dev/ttyUSB0"/>
     <param name="serial_baudrate"  type="int"    value="115200"/>
     <param name="frame_id"         type="string" value="horizontal_laser_link"/>
     <param name="inverted"         type="bool"   value="false"/>
     <param name="angle_compensate" type="bool"   value="true"/>
  </node>
  <node
      pkg="tf"
      type="static_transform_publisher"
      name="base_link_connect"
      args="0 0 0 0 0 0 /base_link /horizontal_laser_link 100"
  />
  <node
      pkg="tf"
      type="static_transform_publisher"
      name="imu_link_connect"
      args="0 0 0 0 0 0 /base_link /imu_link 100"
  />
  <node
      name="cartographer_node"
      pkg="cartographer_ros"
      type="cartographer_node"
      args="-configuration_directory $(find cartographer_ros)/configuration_files -configuration_basename backpack_2d.lua"
      output="screen">
  </node>
  <node
      name="cartographer_occupancy_grid_node"
      pkg="cartographer_ros"
      type="cartographer_occupancy_grid_node"
      args="-resolution 0.05"
  />
</launch>
```
9. Execute below.
```
$ cd ~/catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files
$ cp backpack_2d.lua BK_backpack_2d.lua
$ nano backpack_2d.lua
```
```
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.
TRAJECTORY_BUILDER_2D.max_range = 20.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.huber_scale = 1e3

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

return options
```
10. Execute below.
```
$ cd ~/catkin_ws
$ catkin_make_isolated --install --use-ninja
$ source install_isolated/setup.bash
```

## ◆ Execution and visualization of map generation by RaspberryPi + Arduino + RPLidar A1M8<br>

### **Perform work with RaspberryPi3 (Raspbian Stretch)**<br>

**Terminal-1**
```
$ roscore
```
**Terminal-2**
```
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```
**Terminal-3**
```
$ sudo rfcomm listen /dev/rfcomm0 1
```

<br>
Windows 10 Launch TeraTerm on PC side and connect to RFCOMM to RaspberryPi.
<br>

**Terminal-4**
```
$ rosrun zumo32u4 zumo32u4.py
```
**Terminal-5**
```
$ cd ~/catkin_ws
$ source install_isolated/setup.bash
$ roslaunch cartographer_ros backpack_2d.launch
```
### **Perform work with Ubuntu16.04**<br>

**Terminal-1**
```
$ cd ~/catkin_ws
$ catkin_make_isolated --install --use-ninja
$ source install_isolated/setup.bash
$ export ROS_MASTER_URI=http://raspberrypi.local:11311/
$ roslaunch cartographer_ros backpack_2d.launch
```

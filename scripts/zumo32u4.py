#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import traceback
import serial
from math import sqrt, cos, sin
import rospy
import tf
from time import sleep
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Zumo:
    def __init__(self):
        self.DIAMETER=0.039              #[Meter]  Diameter of tire
        self.INTERAXIS=0.084             #[Meter]  Distance between left and right tires
        self.COUNT=12                    #[Count]  Resolution of encoder
        self.RADIANPERENCODER=0.00121043 #[Radian] Radian per encoder count 1
        self.temps=0.0                   #[Second] Last measured time
        self.delta=0.0                   #[Second] Elapsed time
        self.theta=0.0                   #[Radian] Radian
        self.command=""                  #[String] Robot operation command

        try:
            self.PORT = rospy.get_param('BLUETOOTH_PORT') 
        except:
            rospy.set_param('BLUETOOTH_PORT',"/dev/rfcomm0")
            self.PORT = rospy.get_param('BLUETOOTH_PORT')

        try:
            self.BAUDRATE = rospy.get_param('BLUETOOTH_BAUDRATE') 
        except:
            rospy.set_param('BLUETOOTH_BAUDRATE', "115200")
            self.BAUDRATE = rospy.get_param('BLUETOOTH_BAUDRATE')

        self.TIMEOUT = 0.01
        self.sensorvalue = list()
        self.speed = []
        self.angle = []

        # Init Odometry
        self.o = Odometry()
        self.o.pose.pose.position.x = 0.0
        self.o.pose.pose.position.y = 0.0
        self.o.pose.pose.position.z = 0.0
        self.o.pose.pose.orientation.z = 0.0        
        self.o.header.stamp = rospy.Time.now()
        self.o.header.frame_id = "odom"
        self.o.child_frame_id = "base_link"

        # Init IMU
        self.p = Imu()
        self.p.header.stamp = rospy.Time.now()
        self.p.header.frame_id = "imu_link"

        try:
            self.ser = serial.Serial(self.PORT, self.BAUDRATE, timeout = self.TIMEOUT)
            sleep(1)
            rospy.loginfo("Serial connection established on the port "+str(self.PORT))
        except:
            rospy.logwarn("Serial connection failure")

        self.pub_comm      = rospy.Publisher('command', String, queue_size=10)
        rospy.loginfo("Publisher initialization success /command")
        self.pub_imu       = rospy.Publisher('imu', Imu, queue_size=10)
        rospy.loginfo("Publisher initialization success /imu")
        self.pub_odom      = rospy.Publisher('odom', Odometry, queue_size=10)
        rospy.loginfo("Publisher initialization success /odom")
        self.sub_sensorval = rospy.Subscriber('sensorval', String, self.subsensorval)
        rospy.loginfo("Subscriber initialization success /sensorval")
        self.tf_br         = tf.TransformBroadcaster()
        rospy.loginfo("TransformBroadcaster initialization success")

    def __delete__(self):
        self.ser.close()

    def pubcommand(self):
        try:
            self.ser.flush()
            self.command = ""
            self.command = self.ser.read()
            if self.command != "":
                rospy.loginfo("Command received ["+self.command+"]")
                self.pub_comm.publish(self.command)
        except:
            pass

    def subsensorval(self, svalue):
        try:
            if len(svalue.data) > 0:
                #rospy.loginfo(svalue.data)
                self.sensorvalue = svalue.data.split(',')
                if len(self.sensorvalue) == 14:
                    self.pubimu()
                    self.pubodom()
        except:
            print "subsensorval Error"
            traceback.print_exc()

    def pubimu(self):
        self.p.linear_acceleration.x=4*9.81*(float(self.sensorvalue[1])/2**16)/100
        self.p.linear_acceleration.y=4*9.81*(float(self.sensorvalue[2])/2**16)/100
        self.p.linear_acceleration.z=4*9.81*(float(self.sensorvalue[3])/2**16)/100
        self.p.orientation.x= float(self.sensorvalue[4])
        self.p.orientation.y=float(self.sensorvalue[5])
        self.p.orientation.z=float(self.sensorvalue[6])
        self.p.header.stamp = rospy.Time.now()
        self.pub_imu.publish(self.p)
    
    def pubodom(self):

        VR=0.0
        VL=0.0

        self.delta=(float(self.sensorvalue[0])-float(self.temps))/1000  #[Second] Elapsed time from latest measurement
        VL=float(self.sensorvalue[9])                                   #[Count] Advance distance of left wheel
        VR=float(self.sensorvalue[10])                                  #[Count] Advance distance of right wheel
        self.temps=self.sensorvalue[0]
        vel = ((VL>0)-(VL<0))*(abs(VL)+abs(VR))/2

        #self.o.pose.pose.position.x += self.delta*(VR+VL)/2*cos(self.theta)
        #self.o.pose.pose.position.y += self.delta*(VR+VL)/2*sin(self.theta)
        #self.theta += self.delta*(VL-VR)/self.INTERAXIS/2*3.14
        self.o.pose.pose.position.x += vel*cos(self.theta)
        self.o.pose.pose.position.y += vel*sin(self.theta)
        if (VL>0 and VR<0) or (VL<0 and VR>0):
            self.theta += vel*self.RADIANPERENCODER
            rospy.loginfo("[VL] "+str(VL)+"[VR] "+str(VR)+"[theta] "+str(self.theta))

        quat = tf.transformations.quaternion_from_euler(0,0,self.theta)

        self.o.pose.pose.orientation.x = quat[0]
        self.o.pose.pose.orientation.y = quat[1]
        self.o.pose.pose.orientation.z = quat[2]
        self.o.pose.pose.orientation.w = quat[3]
#        self.o.twist.twist.linear.x =(VR+VL)/2*cos(self.theta)
#        self.o.twist.twist.linear.y =(VR+VL)/2*sin(self.theta)
#        self.o.twist.twist.angular.z = (VL-VR)/self.INTERAXIS/2*3.14
        self.o.twist.twist.linear.x = vel*cos(self.theta)
        self.o.twist.twist.linear.y = vel*sin(self.theta)
        self.o.twist.twist.angular.z = vel*self.RADIANPERENCODER

        self.o.header.stamp = rospy.Time.now()
        self.pub_odom.publish(self.o)
        
        pos = (self.o.pose.pose.position.x,
               self.o.pose.pose.position.y,
               self.o.pose.pose.position.z)

        ori = (self.o.pose.pose.orientation.x,
               self.o.pose.pose.orientation.y,
               self.o.pose.pose.orientation.z,
               self.o.pose.pose.orientation.w)       
        self.tf_br.sendTransform(pos, ori, rospy.Time.now(), 'base_link', 'odom')

if __name__=="__main__":

    rospy.init_node("zumo", anonymous = True)
    zumo = Zumo()

    while not rospy.is_shutdown():
        zumo.pubcommand()
        sleep(0.001)

    rospy.delete_param("BLUETOOTH_PORT")
    rospy.delete_param("BLUETOOTH_BAUDRATE")
    zumo.ser.close()



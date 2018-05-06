#!/usr/bin/env python
# -*- coding: utf-8 -*-


import serial
from math import sqrt, cos, sin
import rospy
import tf
from time import sleep
from threading import Lock
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class Zumo:
    def __init__(self):
        self.DIAMETER=0.038  #Meter
        self.INTERAXIS=0.084 #Meter
        self.COUNT=48
        self.temps=0
        self.theta=0

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
        self.lock = Lock()
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
        self.o.header.frame_id = "map"
        self.o.child_frame_id = "base_link"

        # Init IMU
        self.p = Imu()
        self.p.header.stamp = rospy.Time.now()
        self.p.header.frame_id = "imu"

        try:
            self.ser = serial.Serial(self.PORT, self.BAUDRATE, timeout = self.TIMEOUT)
            sleep(1)
            rospy.loginfo("Serial connection established on the port "+str(self.PORT))
        except:
            rospy.loginfo("Serial connection failure")

        self.pub_comm      = rospy.Publisher('/zumo32u4/command', String, queue_size=10)
        self.pub_imu       = rospy.Publisher('/zumo32u4/imu', Imu, queue_size=10)
        self.pub_odom      = rospy.Publisher('/zumo32u4/odom', Odometry, queue_size=10)
        self.sub_sensorval = rospy.Subscriber('/zumo32u4/sensorval', String, self.subsensorval)
        self.tf_br         = tf.TransformBroadcaster()

    def __delete__(self):
        self.ser.close()

    def pubcommand(self):
        try:
            if isinstance(self.ser, types.NoneType) == False:
                self.ser.flush()
                command = ""
                command = self.ser.read()
                if command != "":
                    pub.publish(command)
        except:
            print "pubcommand Error"
            print sys.exc_info()
            #pass

    def subsensorval(self, svalue):
        try:
            if len(svalue.data) > 0:
                self.sensorvalue = svalue.data.split(',')
                if len(self.sensorvalue) == 14:
                    self.pubimu()
                    self.pubodom()
        except:
            print "subsensorval Error"
            print sys.exc_info()
            #pass

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
        if self.sensorvalue[10]!=self.odomR or self.sensorvalue[9]!=self.odomL:
            deltat=(float(self.sensorvalue[0])-float(self.temps))/1000                        #Second
            VR=(float(self.sensorvalue[10])-self.odomR)/self.COUNT *3.14*self.DIAMETER/deltat #Meter
            VL=(float(self.sensorvalue[9])-self.odomL)/self.COUNT *3.14*self.DIAMETER/deltat  #Meter
            self.odomL=float(self.sensorvalue[9]) 
            self.odomR=float(self.sensorvalue[10])
            self.temps=self.sensorvalue[0]
        else :
            VR=0
            VL=0        
            
        self.o.pose.pose.position.x += deltat*(VR+VL)/2*cos(self.theta)
        self.o.pose.pose.position.y += deltat*(VR+VL)/2*sin(self.theta)
        self.theta += deltat*(VL-VR)/self.INTERAXIS    
        
        quat = tf.transformations.quaternion_from_euler(0,0,self.theta)

        self.o.pose.pose.orientation.x = quat[0]
        self.o.pose.pose.orientation.y = quat[1]
        self.o.pose.pose.orientation.z = quat[2]
        self.o.pose.pose.orientation.w = quat[3]
        self.o.twist.twist.linear.x =(VR+VL)/2*cos(self.theta)
        self.o.twist.twist.linear.y =(VR+VL)/2*sin(self.theta)
        self.o.twist.twist.angular.z = (VL-VR)/self.INTERAXIS    
        self.o.header.stamp = rospy.Time.now()
        self.pub_odom.publish(self.o)
        
        pos = (self.o.pose.pose.position.x,
               self.o.pose.pose.position.y,
               self.o.pose.pose.position.z)

        ori = (self.o.pose.pose.orientation.x,
               self.o.pose.pose.orientation.y,
               self.o.pose.pose.orientation.z,
               self.o.pose.pose.orientation.w)       
        self.tf_br.sendTransform(pos, ori, rospy.Time.now(), 'base_link', 'map')

if __name__=="__main__":

    rospy.init_node("zumo", anonymous = True)
    zumo = Zumo()

    while not rospy.is_shutdown():
        zumo.pubcommand()
        sleep(0.001)

    rospy.delete_param("BLUETOOTH_PORT")
    rospy.delete_param("BLUETOOTH_BAUDRATE")
    zumo.ser.close()



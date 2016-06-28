#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib;roslib.load_manifest('neu_robotdrive')
import rospyd
from  neu_robotdrive.msg import Num, carOdom #Own definition of the message
from geometry_msgs.msg import Twist

import neu_pyserial as COM_ctr #Write own serial listening module
import glob
from math import sqrt, atan2, pow


class bluetooth_cmd():
    def __init__(self):
        rospy.init_node('robot_driver', anonymous=True)
    
    def callback(self,msg ):

        cmd_twist_rotation =  msg.angular.z #
        cmd_twist_x  = msg.linear.x * 10.0
        cmd_twist_y =  msg.linear.y * 10.0
        
        wheelspeed = self.odom_to_speed(cmd_twist_x, cmd_twist_y,cmd_twist_rotation)
        print 'msg:', msg
        print wheelspeed
        self.blue_tooth_send([wheelspeed[0], wheelspeed[1]])
    
    def odom_to_speed(self, cmd_twist_x =0, cmd_twist_y=0,cmd_twist_rotation=0):
        
        cent_speed = sqrt(pow(cmd_twist_x, 2) + pow(cmd_twist_y, 2))
        yawrate2 = self.yawrate_to_speed(cmd_twist_rotation)
        
        Lwheelspeed = cent_speed - yawrate2/2
        Rwheelspeed = cent_speed + yawrate2/2
        
        return Lwheelspeed, Rwheelspeed
        
    def yawrate_to_speed(self, yawrate):#With the fitting method, the differential speed of the left 
                                        # and the right wheels is converted to the rotational speed, 
                                        #and the speed of the two wheels is multiplied by the coefficient.
            if yawrate > 0:
            theta_to_speed = 0.0077 #Right turn coefficient
        else:
            theta_to_speed = 0.0076  #Left turn coefficient
            
        x = (yawrate * 0.02) / theta_to_speed #Sampling yawrate：rad/s *0.02 said yawrate should turn the number of arc,
                                              # /0.0076 is to turn the arc into the right and left wheel speed difference
        return   x
        
    def talker(self):
        self.rec_data = COM_ctr.SerialData( datalen = 2)  #Start listening COM thread
        allport = glob.glob('/dev/ttyU*')
        port = allport[0]
        baud = 115200
        openflag = self.rec_data.open_com(port, baud)
        
        rospy.Subscriber("/cmd_vel", Twist, self.callback)#subscribe Control command by move_base
        
        pub = rospy.Publisher('car_speed', carOdom)
        pub_wheel = rospy.Publisher('wheel_speed', Num) #Left and right wheel speed
        
        r = rospy.Rate(500) # 100hz
        Lwheelpwm= 0
        
        sumL = 0
        sumR = 0
        
        while not rospy.is_shutdown():
            all_data = []
            if self.rec_data.com_isopen():
                all_data = self.rec_data.next()  #Received data set
       
            if all_data != []:  #If you do not receive the data, do not perform the followings
                wheelspeed = Num()  #Own definition of the message
                car_speed = carOdom()
                leftspeed = all_data[0][0]
                rightspeed = all_data[1][0]
                wheelspeed.leftspeed = leftspeed
                wheelspeed.rightspeed = rightspeed
                #The speed of the left and right wheel is converted to the speed of the X axis 
                #of the robot and the speed of the rotation of the Z axis.
                resluts = self.speed_to_odom(leftspeed, rightspeed)
                car_speed.x = resluts[0]
                car_speed.y = resluts[1]
                car_speed.vth = resluts[2]

                pub.publish(car_speed)
                pub_wheel.publish(wheelspeed)
                
            r.sleep()
         
        if openflag:
            self.rec_data.close_lisen_com()  
    
    def speed_to_odom(self, Lspeed = 0, Rspeed = 0):
        delta_speed = Rspeed - Lspeed
        if delta_speed < 0:
            theta_to_speed = 0.0077 #Right turn coefficient
        else:
            theta_to_speed = 0.0076  #Left turn coefficient
            
        v_th = delta_speed  * theta_to_speed / 0.02    # first : transform delta_speed to  delta_theta .   second: dived by delta_t 								(20ms), get the yawrate
        v_x = (Rspeed + Lspeed)/10.0/2.0    # Lspeed : dm/s   -- > m/s  so need to /10.0
        v_y = 0.0
        return v_x, v_y, v_th
        
    def blue_tooth_send(self, data = [], head = 'HY'):
        if data !=[] and self.rec_data.com_isopen():
            self.rec_data.send_data(data, head)   #Around the central axis rotation is set to 0
#        print data
            
if __name__ == '__main__':
    try:
        car_cmd = bluetooth_cmd()
        car_cmd.talker()
    except rospy.ROSInterruptException: pass


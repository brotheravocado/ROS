#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist,Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2


class func:

    def __init__(self):
        self.cleanmode = True
        self.searchingmode = False
        self.scan_sub = rospy.Subscriber('laserscan',LaserScan, self.Scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        rospy.init_node('vaccums')
        self.right = 1
        self.left = 1
        self.ahead = 1
        self.roll = self.pitch = self.yaw = 0.0
        self.twist = Twist()
        self.turn_twist = Twist()
        self.state_change_time = rospy.Time.now()
        self.rotating = True
        self.wallX = list()
        self.wallY = list()
        self.poseX = 0
        self.poseY = 0
        self.afewX = round(self.poseX, 1)
        self.afewY = round(self.poseY, 1)
        self.walllistcount = 0
        self.tartget = 90
        self.kp = 1.0
        self.location_List = []
        self.cleaned_list = []
        self.positionlist = [[0, 0]]
        self.cleanedX = [0]
        self.cleanedY = [0]
        self.cleanedlistcount = 0
        self.north = False
        self.south = False
        self.west = False
        self.east = False
        self.finish_check = False
        self.area = 0
        print("It'S function Class")

    def start(self):  # start

        self.set_left(182)
        time.sleep(1.3)
        self.Testing()


    def Scan_callback(self,msg):
        self.ahead = round(min(msg.ranges[130:140]),2)
        self.right = max(msg.ranges[43:70])
        self.left = max(msg.ranges[223:250])
        self.left2=msg.ranges[180]
        self.real_left = msg.ranges[225]
        self.real_right = msg.ranges[45]


        #print 'range_ahead = %.1f' % self.ahead, 'left = %.1f' % self.left, 'right = %.1f' % self.right

    def odom_callback(self,msg):
        self.poseX = round(msg.pose.pose.position.x, 1)
        self.poseY = round(msg.pose.pose.position.y, 1)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
        #print 'Position X = %.1f' % self.poseX, 'Position Y = %.1f' % self.poseY



    def Testing(self):

        self.print_status()
        while not rospy.is_shutdown():

            if self.finish_check == True:
                break

            self.move_north_east()

            if self.north == True and self.real_right < 5:
                self.cleanning1()
            elif self.south == True and self.real_left < 5:
                self.cleanning1()
            elif (self.north == True and self.real_right >= 5) or (self.south == True and self.real_left >= 5):
                print("checkcheckcheck")
                self.moving_south_east()
                print("checkcheckcheck")
                self.cleanning2()

            self.searching()




    def move_north_east(self):
        print("moving north_east")

        while not rospy.is_shutdown():
            self.check_Yaw()
            if self.north == True and self.ahead > 0.8:
                self.search_go_afew()
            elif self.north == True and self.ahead <= 0.8 and self.real_left >= 0.8:
                self.set_left(90)
            elif self.north == True and self.ahead <= 0.8 and self.real_left < 0.8:
                self.set_left(182)
            elif self.west == True and self.ahead > 0.8:
                self.search_go_afew()
            elif self.west == True and self.ahead <= 0.8:
                self.set_left(182)
                self.stop_move()
            elif self.south == True:
                self.cleanmode = True
                ("ready clean")
                break

    def moving_south_east(self):
        self.set_left(182)
        time.sleep(1.3)
        self.go_ahead()
        self.set_left(98)
        time.sleep(1.3)
        self.go_ahead()



    def finish_mode(self):
        self.set_left(96)
        time.sleep(1.3)
        self.clean_go_ahead()
        time.sleep(1.3)
        self.set_left(-87)
        time.sleep(1.3)
        self.set_left(2)
        time.sleep(1.3)
        self.clean_go_ahead()
        self.finish_check = True
        print("[----------------------------------------]")
        print("[                  Finish                ]")
        print("[----------------------------------------]")

    def print_status(self):
        self.check_Yaw()
        print 'range_ahead = %.1f' % self.ahead, 'left = %.1f' % self.left, 'right = %.1f' % self.right, 'real_left= %.1f' % self.real_left
        # print 'Position X = %.1f' % self.poseX, 'Position Y = %.1f' % self.poseY
        print("north", self.north, "south", self.south, "east", self.east, "west", self.west)


    def cleanning1(self):
        print("[----------------------------------------]")
        print("[-------------Clean mode 1---------------]")
        print("[----------------------------------------]")
        count = 0
        while not rospy.is_shutdown():
            if self.cleanmode == True:
                self.clean_go_ahead()

                self.set_left(-90)
                time.sleep(1.3)
                self.clean_go_afew()
                #self.clean_go_afew()
                time.sleep(1.3)
                self.set_left(2)
                time.sleep(1.3)
                self.clean_go_ahead()
                time.sleep(1.3)
                self.set_left(-90)
                time.sleep(1.3)
                self.clean_go_afew()
                time.sleep(1.3)
                self.set_left(182)
                # count = count + 1
                # print("count = ", count)
                time.sleep(1.3)



            else :
                break


    def cleanning2(self):
        print("[----------------------------------------]")
        print("[-------------Clean mode 2---------------]")
        print("[----------------------------------------]")
        count = 0
        while not rospy.is_shutdown():
            self.set_left(2)
            time.sleep(1.3)

            self.go_afew()
            time.sleep(1.3)
            self.set_left(-87)
            time.sleep(1.3)
            self.clean_go_ahead()
            time.sleep(1.3)
            self.set_left(2)

            time.sleep(1.3)
            self.go_afew()
            count += 1
            print("count = ", count)
            if count == 2:
                self.clean_go_afew()
                time.sleep(1.3)
                print("finish_Mode")
                self.clean_go_afew()
                self.finish_mode()
                break
            time.sleep(1.3)
            self.set_left(96)
            time.sleep(1.3)
            self.clean_go_ahead()
            time.sleep(1.3)


    def cleaned_area(self):
        self.area = round(self.cleanedlistcount/5 * 25 * 25 * 3.14 , 1)/10000
        print("cleaend area is  {0} m^2 ]" .format(self.area) )


    def searching(self):
        print("[----------------------------------------]")
        print("[------------Searching mode -------------]")
        print("[----------------------------------------]")
        while not rospy.is_shutdown():
            if self.searchingmode == True:
                self.check_Yaw()
                self.print_status()
                if self.east == True and self.ahead <= 0.9 and self.real_left <= 1.2:
                    self.set_left(182)
                    self.stop_move()
                elif self.south == True and self.ahead >= 0.5 and self.real_left <= 1.2:
                    self.search_go_afew()
                elif self.south == True and self.ahead <= 0.5 and self.real_left >= 2:
                    self.set_left(-90)
                    self.stop_move()
                elif self.east == True and self.ahead >= 2.0:
                    self.search_go_afew()
                    self.print_status()
                elif self.east == True and self.ahead >= 1.6 :
                    self.set_left(2)
                elif self.north == True and self.ahead > 1.0 and self.left2 < 1.5:
                    self.search_go_afew()

                elif self.south == True and self.ahead <= 0.9 and self.real_left < 1.2:
                     self.set_left(-87)
                     time.sleep(1.3)
                     self.set_left(96)
                     print"check"
                elif self.west == True and self.ahead >= 1.0 and self.real_left <= 1.5:
                    self.search_go_afew()
                elif self.west == True  and self.ahead <= 1.0 and self.real_left <= 1.5:
                    self.set_left(2)

            if self.real_left < 1.5 and self.ahead < 1.5 and self.north == True:
                print("searching finish")
                self.searchingmode == False
                break

    def left_rotate(self, target):  # turn lefta

        target_rad = target * math.pi / 180
        time.sleep(1.3)
        while not rospy.is_shutdown():
            self.turn_twist.angular.z = self.kp * (target_rad - self.yaw)
            self.cmd_vel_pub.publish(self.turn_twist)

            # print(self.yaw)
            if (self.yaw >= 3.14 and target == 182):
                print("turn_left (180 degree)  out")
                break

            elif (self.yaw > 1.52 and target == 91):
                print("turn_left (91 degree)  out")
                break
            elif (self.yaw <= -1.5 and target == -87):
                print("turn_left (87 degree)  out")
                break

            elif (self.yaw > 1.5 and target == 82):
                print("turn_left (90 degree)  out")
                break

            elif (self.yaw > 1.52 and target == 98):
                print("turn_left (98 degree)  out")
                break
            elif (self.yaw > 1.52 and target == 96):
                print("turn_left (96 degree)  out")
                break


            elif (self.yaw > 1.52 and target == 90):
                print("turn_left (95 degree)  out")
                break

            elif (self.yaw > 0.031 and target == 2):
                print("turn_left (2) out")
                break

            elif (self.yaw < -1.5 and target == -90):
                print("turn_left (90 degree)  out")
                break

    def set_left(self, i=90):  # turn left
        self.left_rotate(i)

    def clean_go_ahead(self):
        # self.beforepositionlist[0] = [self.poseX, self.poseY]
        time.sleep(1.3)
        while not self.ahead <= 0.9 or not rospy.is_shutdown():
            if not self.ahead < 0.5:
                self.twist.linear.x = 0.5
                self.cmd_vel_pub.publish(self.twist)
                self.cleaned_location()
                self.check_Yaw()
            else:
                self.stop_move()
                self.cleaned_location()
                self.check_Yaw()
                break
            if self.east == True and (self.ahead >= 0 and self.ahead <= 0.8):
                print(self.ahead)
                self.cleaned_location()
                self.check_Yaw()
                print(self.ahead)
                self.cleanmode = False
                self.searchingmode = True
                print("cleanmode end")
                print("go_ahead cleanmode end")
                time.sleep(1.0)

    def search_go_ahead(self):

        # self.beforepositionlist[0] = [self.poseX, self.poseY]
        while not self.ahead <= 0.5 or not rospy.is_shutdown():
            if not self.ahead < 0.5:
                self.twist.linear.x = 0.5
                self.cmd_vel_pub.publish(self.twist)
                self.cleaned_location()
                self.check_Yaw()
            else:
                self.stop_move()
                self.cleaned_location()
                self.check_Yaw()
                break



    def check_Yaw(self):
        yaw = float('{0:01.1f}'.format(self.yaw))
        if yaw <= -1.4 and yaw >= -1.8:  # or (yaw>=-3.2 and yaw<=2.9):
            # print("East")
            self.north = False
            self.south = False
            self.west = False
            self.east = True
        elif yaw >= 1.4 and yaw <= 1.8:  # or (yaw<=-0 and yaw >= -0.2) :
            # print("West")
            self.north = False
            self.south = False
            self.west = True
            self.east = False
        elif yaw >= -0.2 and yaw <= 0.2:
            # print("North")
            self.north = True
            self.south = False
            self.west = False
            self.east = False
        elif  (yaw >= 2.9 and yaw <= 3.2) or (yaw <= -2.9 and yaw >= -3.2):
            # print("South")
            self.north = False
            self.south = True
            self.west = False
            self.east = False

    def detect_wall(self):
        self.wallX.append(self.poseX)
        self.wallY.append(self.poseY)
        print (self.wallX[self.walllistcount]), (self.wallY[self.walllistcount])
        self.walllistcount = self.walllistcount + 1

    def check_Yaw(self):
        yaw = float('{0:01.1f}'.format(self.yaw))
        if yaw <= -1.4 and yaw >= -1.8:  # or (yaw>=-3.2 and yaw<=2.9):
            # print("East")
            self.north = False
            self.south = False
            self.west = False
            self.east = True
        elif yaw >= 1.4 and yaw <= 1.8:  # or (yaw<=-0 and yaw >= -0.2) :
            # print("West")
            self.north = False
            self.south = False
            self.west = True
            self.east = False
        elif yaw >= -0.2 and yaw <= 0.2:
            # print("North")
            self.north = True
            self.south = False
            self.west = False
            self.east = False
        elif (yaw >= 2.9 and yaw <= 3.2) or (yaw <= -2.9 and yaw >= -3.2):
            # print("South")
            self.north = False
            self.south = True
            self.west = False
            self.east = False

    def clean_go_afew(self):
        # self.beforepositionlist[0] = [self.poseX, self.poseY]
        self.check_Yaw()
        if self.east == True:
            self.afewY = self.poseY - round(0.3, 1)
            while self.poseY > self.afewY:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break
        elif self.north == True:
            self.afewX = self.poseX + round(0.1, 1)
            print("before", self.afewX)
            while self.poseX < self.afewX:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

        elif self.south == True:
            self.afewX = self.poseX - round(0.1, 1)
            while self.poseX > self.afewX:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

        elif self.west == True:
            self.afewY = self.poseY + round(0.1, 1)
            while self.poseY < self.afewY:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

        if self.east == True and (self.ahead >= 0 and self.ahead <= 1.0):
            self.cleaned_location()
            self.check_Yaw()
            self.cleanmode = False
            self.searchingmode = True
            print("go_afew cleanmode end")
            time.sleep(1.0)

    def search_go_afew(self):
        # self.beforepositionlist[0] = [self.poseX, self.poseY]
        self.check_Yaw()
        if self.north == True:
            self.afewX = self.poseX + round(0.1, 1)
            print("before", self.afewX)
            while self.poseX < self.afewX:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

        elif self.south == True:
            self.afewX = self.poseX - round(0.1, 1)
            while self.poseX > self.afewX:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

        elif self.west == True:
            self.afewY = self.poseY + round(0.1, 1)
            while self.poseY < self.afewY:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

        elif self.east == True:
            self.afewY = self.poseY - round(0.1, 1)
            while self.poseY > self.afewY:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.3
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

    def go_ahead(self):
        while not self.ahead <= 0.4:
            if not self.ahead < 0.4:
                self.twist.linear.x = 0.2
                self.cmd_vel_pub.publish(self.twist)
                self.cleaned_location()

    def go_afew(self):
        self.check_Yaw()
        if self.north == True:
            self.afewX = self.poseX + round(0.4, 1)
            print("before", self.afewX)
            while self.poseX < self.afewX:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

        elif self.south == True:
            self.afewX = self.poseX - round(0.4, 1)
            while self.poseX > self.afewX:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

        elif self.west == True:
            self.afewY = self.poseY + round(0.4, 1)
            while self.poseY < self.afewY:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.5
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

        elif self.east == True:
            self.afewY = self.poseY - round(0.4, 1)
            while self.poseY > self.afewY:  # move x
                if not self.ahead < 0.5:
                    self.twist.linear.x = 0.3
                    self.cmd_vel_pub.publish(self.twist)
                    self.cleaned_location()
                    self.check_Yaw()
                else:
                    self.stop_move()
                    self.cleaned_location()
                    self.check_Yaw()
                    break

    def cleaned_location(self):
        # if self.poseX not in self.cleanedX or self.poseY not in self.cleanedY:
        self.positionlist[0] = [self.poseX, self.poseY]
        if self.positionlist[0] not in self.cleaned_list:
            self.cleanedX.append(self.poseX)
            self.cleanedY.append(self.poseY)
            self.cleaned_list.append([self.cleanedX[self.cleanedlistcount], self.cleanedY[self.cleanedlistcount]])
            print (self.cleanedlistcount), (self.cleanedX[self.cleanedlistcount]) , (self.cleanedY[self.cleanedlistcount]), (self.cleaned_list[self.cleanedlistcount]),(self.positionlist[0])
            self.cleanedlistcount = self.cleanedlistcount + 1

    def stop_move(self):  # stop
        self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def print_status(self):
        self.check_Yaw()
        print 'range_ahead = %.1f' % self.ahead, 'left = %.1f' % self.left, 'right = %.1f' % self.right, 'real_left= %.1f' % self.real_left
        # print 'Position X = %.1f' % self.poseX, 'Position Y = %.1f' % self.poseY
        print("north", self.north, "south", self.south, "east", self.east, "west", self.west)
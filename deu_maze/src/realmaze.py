#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from mazestack import Stack
from go_base import move_base


class run:
    def __init__(self):
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        rospy.init_node('maze')
        self.dis_x = 0
        self.dis_y = 0
        self.right = 5
        self.left = 5
        self.ahead = 5
        self.path_count = 0
        self.twist = Twist()
        self.turn_twist = Twist()
        self.state_change_time = rospy.Time.now() #현재시간저장
        self.st = Stack()
        self.stack_yaw = 0.0
        self.flag = False
        self.first = True
        self.check = True

    def pose_callback(self, msg): # odometry 토픽을 이용한 callback 함수
        self.goal_x = 15.8 # 도착좌표
        self.goal_y = -10.3
        self.x = msg.pose.pose.position.x # 현재좌표
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.dis_x = abs(self.goal_x - self.x) # 현재좌표와 반환좌표와의 거리
        self.dis_y = abs(self.goal_y - self.y)

    def scan_callback(self, msg): #laserscan 토픽callback 함수
        self.ahead = round(min(msg.ranges[850:950]), 2) # 정면 센서
        self.right = max(msg.ranges[1105:1350]) #오른쪽센서
        self.left = max(msg.ranges[450:725]) #왼쪽센서


        
    def check_path(self):
        if (float('{0:01.1f}'.format(self.ahead))) >= 3.0 and ( # 갈수있는경로가2개인경우
                (float('{0:01.1f}'.format(self.right)) >= 2.5 and float('{0:01.1f}'.format(self.left)) <= 1.0) or (
                float('{0:01.1f}'.format(self.left)) >= 2.5 and float(
            '{0:01.1f}'.format(self.right)) <= 1.0)) and self.check is True:

            self.check = False
            self.path_count = 2
            self.change_yaw()
            self.st.push(
                [float('{0:01.1f}'.format(self.x)), float('{0:01.1f}'.format(self.y)), self.path_count,
                 self.stack_yaw])
            self.print_pushview()

            if self.first == False:
                self.stop_move()
                time.sleep(1.0)
            else:
                self.first = False

        if self.right > 1.5 and self.left > 1.5: #갈수있는경로가2개인경우
            self.path_count = 2
            # print "2 path check"
        elif self.right > 1.5 or self.left > 1.5:#갈수있는경로가1개인경우
            self.path_count = 1

        return self.path_count

    def start(self):  # start
        print "start"
        # self.path_count = 1
        self.st.push(
            [0.0, 0.0, 2, 1.5])
        self.turn_right()

    def turn_right(self):  # turn right
        time.sleep(1.0)
        self.turn_twist.angular.z = -90 * math.pi / 180
        self.angle = self.yaw - math.pi / 2

        if self.flag == False: #push의경우
            print "right"
            self.print_pushview()
        else:
            self.print_popview() #pop의경우
        if abs(self.angle) > math.pi:
            self.angle += 2 * math.pi
        while abs(self.yaw - self.angle) != 0:
            self.cmd_vel_pub.publish(self.turn_twist)
        self.stop_move() #정지
        self.check = True
        self.change_yaw() #push를 위한yaw값변경
        self.st.push(
            [float('{0:01.1f}'.format(self.x)), float('{0:01.1f}'.format(self.y)), self.path_count, self.stack_yaw]) #push

    def turn_left(self):  # turn left
        time.sleep(1.0)

        self.turn_twist.angular.z = 90 * math.pi / 180
        self.angle = self.yaw + math.pi / 2
        if self.flag == False: #push의경우
            print "left"
            self.print_pushview()
        else:
            self.print_popview() #pop의경우
        if abs(self.angle) > math.pi:
            self.angle -= 2 * math.pi
        while abs(self.yaw - self.angle) != 0:
            self.cmd_vel_pub.publish(self.turn_twist)
        self.stop_move() #정지
        self.change_yaw() #push를 위한yaw값변경
        self.check = True
        self.st.push(
            [float('{0:01.1f}'.format(self.x)), float('{0:01.1f}'.format(self.y)), self.path_count, self.stack_yaw]) #push

    def ahead_is_small(self): #정면센서의값이0.8보다작은경우
        self.stop_move()
        if self.left <= 1.5 and self.right <= 1.5 and self.state_change_time < rospy.Time.now():
            self.flag = True
            self.turn_right()
            self.state_change_time = rospy.Time.now() + rospy.Duration(2)

        elif self.check_path() == 2:
            time.sleep(0.1)
            self.check_path()  # add
            self.flag = False
            self.turn_left()
            self.state_change_time = rospy.Time.now() + rospy.Duration(3)

    def right_is_big(self): #오른쪽센서가 왼쪽센서보다 큰 경우
        if self.ahead <= 2.4 and self.ahead > 0.8:
            while self.ahead >= 0.8:
                self.twist.linear.x = 1.0
                self.cmd_vel_pub.publish(self.twist)
        else:

            self.turn_left()
            self.stop_move()
            self.state_change_time = rospy.Time.now() + rospy.Duration(3)

    def left_is_big(self): #왼쪽센서가 오른쪽센서보다 큰 경우
        if self.ahead <= 2.4 and self.ahead > 0.8: #만약 정면센서의 값이 너무크다면 값조정을 위해 직진주행
            while self.ahead >= 0.8:
                self.twist.linear.x = 1.0
                self.cmd_vel_pub.publish(self.twist)
        else:
            self.check_path()
            self.turn_right()
            self.stop_move()
            self.state_change_time = rospy.Time.now() + rospy.Duration(3)

    def loop(self): #반복

        if not self.ahead <= 0.8: #정면센서가 0.8보다  큰경우

            self.go_straight()

        else: #정면센서가 0.8보다  작은경우

            self.stop_move()


            self.ahead_is_small()

        if self.right >= 2.0 and self.left <= 1.5 and self.state_change_time < rospy.Time.now(): # 오른쪽이 2.0보다 크고 왼쪽이 1.5보다 작은경우
            self.right_is_big()
        if self.left >= 2.0 and self.right <= 1.5 and self.state_change_time < rospy.Time.now(): # 왼쪽이 2.0보다 크고 오른쪽이 1.5보다 작은경우
            self.left_is_big()
        if self.flag == True:  # pop
            if self.st.is_empty() == False: #스택이 비어있는지 확인
                self.pop_stack()

    def pop_stack(self):  # pop
        list = self.st.peek()

        if list[2] == 2:
            d_x = abs(list[0] - float('{0:01.1f}'.format(self.x)))
            d_y = abs(list[1] - float('{0:01.1f}'.format(self.y)))
            if d_x <= 0.2 and d_y <= 0.2: #현재 터틀봇과의거리가 0.2보다작다면
                self.st.pop() #pop
                self.change_yaw()
                self.st.push( #push
                    [float('{0:01.1f}'.format(self.x)), float('{0:01.1f}'.format(self.y)), self.check_path(),
                     self.stack_yaw])
                self.flag = False #pop 중지

        elif list[2] == 1: # 갈수있는 경로가 1이라면
            self.st.pop() #pop

    def go_straight(self):  # go straight
        self.check_path()
        self.twist.linear.x = 1.0
        self.cmd_vel_pub.publish(self.twist)

    def stop_move(self):  # stop
        self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)


    def goal_check(self): #반환점에 도착하였는지 확인
        if self.dis_x <= 0.2 and self.dis_y <= 0.2:
            self.stop_move()
            return False
        elif self.dis_x > 0.2 or self.dis_y > 0.2:
            return True

    def back(self): #반환점도착시 되돌아가는 클래스호출
        self.stop_move()
        repath = move_base(self.st)
        repath.follow()

    def print_pushview(self): # pushview
        print "push - self.x : ", self.x, " self.y : ", self.y

    def print_popview(self): #pop view
        print "pop"

    def change_yaw(self): #push시 되돌아오는 yaw값계산
        self.stack_yaw = float('{0:01.1f}'.format(self.yaw))
        if self.stack_yaw >= 0 and self.stack_yaw <= 3.1:
            if self.stack_yaw == 0:
                self.stack_yaw += 3.1
            elif self.stack_yaw == 3.1:
                self.stack_yaw = 0
            else:
                self.stack_yaw = 3.1 - self.stack_yaw
                self.stack_yaw = - abs(self.stack_yaw)
        elif self.stack_yaw >= -3.1 and self.stack_yaw <= 0:
            if self.stack_yaw == 0:
                self.stack_yaw += 3.1
            elif self.stack_yaw == -3.1:
                self.stack_yaw = 0
            else:
                self.stack_yaw = 3.1 + self.stack_yaw
                self.stack_yaw = abs(self.stack_yaw)


    def error_stop(self): #정면센서가 0.1보다작다면 error
        if self.ahead <= 0.1:
            print "error"
            return False
        else:
            return True






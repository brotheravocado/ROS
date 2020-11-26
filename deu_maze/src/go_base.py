#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist


class move_base: # 반환점 도착 시 stack을 통해 pop하며 출발점으로 되돌아가는 함수
    def __init__(self, waypoints):
        self.x = 0 # 로봇의 위치
        self.y = 0
        self.yaw = 0 # 로봇의 방향
        self.inc_x = 0
        self.inc_y = 0
        self.goal_x = 0.0 # 출발 지점 좌표
        self.goal_y = 0.0
        self.ahead = 5 # 로봇의 정면 센서
        self.l = waypoints  # 반환점 도착 시 최단 경로 스택
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.pose_sub = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self.twist = Twist()


    def scan_callback(self, msg): #laserscan 토픽callback 함수
        self.ahead = round(min(msg.ranges[850:950]), 2)


    def pose_callback(self, msg): # odometry 토픽을 이용한 callback 함수
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def right_rotate(self):  #  오른쪽으로 회전
        while True:
            self.twist.angular.z = -1.0
            self.pub.publish(self.twist)
            if abs(self.yaw - self.point[3]) <= 0.1: # 로봇의 yaw값과 pop한 list의 yaw값의 차이가 0.1보다 작거나 같을 때까지 회전한
                break
        self.stop_move() # 회전 후 정지
        print "stop and go"

    def error_stop(self):  # 정면센서의 값이 0.1보다 작다면 error를 출력하고 종료
        if self.ahead <= 0.1:
            print "error"
            return False
        else:
            return True

    def follow(self): # 최단경로로 돌아가기위한 follow operation
        while not self.l.is_empty() and self.error_stop(): # stack이 비지않고 정면 센서의 값이 0.1보다 큰 경우에만 while 문 반복
            self.point = self.l.pop()  # stack을 pop한 list를 point로 대입
            print "pop"
            self.right_rotate()

            while True:
                self.inc_x = abs(self.point[0] - self.x) # list의 좌표와 현재 터틀봇의 좌표의 거리 차이
                self.inc_y = abs(self.point[1] - self.y)
                if self.point[3] == 0.0 or self.point[3] == 3.1: # list의 yaw 값이 0.0,3.1인 경우
                    if self.inc_x <= 0.1: # list의 x좌표와 현재 터틀봇의 x좌표의 거리가 0.1보다 작거나 같은경우
                        self.stop_move() # 정지 후 break를 걸어 다음 스택을 pop한다.
                        print "inc_x < 0.1"
                        if self.point[0] == self.goal_x :
                            print "goal"
                            self.stop_move()
                        break

                    elif self.inc_x > 0.1: # list의 좌표와 현재 터틀봇의 좌표의 거리가 0.1보다 큰경우 직진 주행한다.
                        self.twist.linear.x = 1.0
                        self.twist.angular.z = 0.0
                        self.pub.publish(self.twist)

                elif self.point[3] == 1.5 or self.point[3] == -1.5: # list의 yaw 값이 1.5,-1.5인 경우
                    if self.inc_y <= 0.1: # list의 x좌표와 현재 터틀봇의 y좌표의 거리가 0.1보다 작거나 같은경우
                        self.stop_move() # 정지 후 break를 걸어 다음 스택을 pop한다.
                        print "inc_y < 0.1"
                        if self.point[1] == self.goal_y:
                            print "goal"
                            self.stop_move()
                        break
                    elif self.inc_y > 0.1: # list의 좌표와 현재 터틀봇의 좌표의 거리가 0.1보다 큰경우 직진 주행한다.
                        self.twist.linear.x = 1.0
                        self.twist.angular.z = 0.0
                        self.pub.publish(self.twist)
        print "finish"

    def stop_move(self):  # stop
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self.pub.publish(self.twist)



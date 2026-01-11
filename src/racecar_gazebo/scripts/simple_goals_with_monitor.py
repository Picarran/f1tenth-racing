#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
三目标点导航 - 通过监控实际位置判断到达
不依赖move_base的状态反馈，而是监控小车真实位置
"""

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
import math

# ============ 在这里设置你的四个目标点 ============
# 角度：单位是弧度：0=向右, 1.57=向上, 3.14=向左, -1.57=向下
# TODO：这个角度还不是很对，到终点会摆动
GOAL_1 = (20.0, 5.2, 0.0)
GOAL_2 = (1.3, 8.5, -1.57)
GOAL_3 = (-5.8, -8.6, -1.57)
GOAL_4 = (-5.8, -8.6, -1.57)
# =================================================

# 到达目标的距离阈值（米）
REACH_THRESHOLD = 0.5

class MultiGoalNavigator:
    def __init__(self):
        self.current_position = None
        self.position_updated = False
        
        # 订阅位置信息
        rospy.Subscriber('/pf/pose/odom', Odometry, self.odom_callback, queue_size=1)
        
        # 连接move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        self.client.wait_for_server()
        rospy.loginfo("已连接！")
    
    def odom_callback(self, msg):
        """接收小车当前位置"""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        self.position_updated = True
    
    def distance_to_goal(self, goal_x, goal_y):
        """计算到目标点的距离"""
        if self.current_position is None:
            return float('inf')
        
        dx = self.current_position[0] - goal_x
        dy = self.current_position[1] - goal_y
        return math.sqrt(dx*dx + dy*dy)
    
    def create_goal(self, x, y, yaw):
        """创建目标点"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        quaternion = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*quaternion)
        
        return goal
    
    def send_and_wait_goal(self, goal_x, goal_y, goal_yaw, goal_num):
        """发送目标并等待到达"""
        rospy.loginfo("=" * 50)
        rospy.loginfo("发送目标点%d: (%.2f, %.2f, %.2f)" % (goal_num, goal_x, goal_y, goal_yaw))
        
        # 发送目标
        goal = self.create_goal(goal_x, goal_y, goal_yaw)
        self.client.send_goal(goal)
        
        # 等待位置更新
        rospy.sleep(0.5)
        
        # 循环检查是否到达
        rate = rospy.Rate(10)  # 10Hz检查频率
        timeout = rospy.Time.now() + rospy.Duration(120.0)  # 2分钟超时
        
        while not rospy.is_shutdown():
            if rospy.Time.now() > timeout:
                rospy.logwarn("目标点%d 超时！" % goal_num)
                return False
            
            # 计算距离
            distance = self.distance_to_goal(goal_x, goal_y)
            
            if distance < REACH_THRESHOLD:
                rospy.loginfo("✓ 目标点%d 已到达！(距离: %.3fm)" % (goal_num, distance))
                return True
            
            # 每5秒报告一次进度
            if int(rospy.Time.now().to_sec()) % 5 == 0:
                if self.current_position:
                    rospy.loginfo("当前位置: (%.2f, %.2f), 距目标: %.2fm" % 
                                (self.current_position[0], self.current_position[1], distance))
            
            rate.sleep()
        
        return False
    
    def navigate(self):
        """执行四点导航"""
        goals = [GOAL_1, GOAL_2, GOAL_3, GOAL_4]
        
        for i, (x, y, yaw) in enumerate(goals, 1):
            if self.send_and_wait_goal(x, y, yaw, i):
                rospy.loginfo("停顿1秒...")
                rospy.sleep(1.0)
            else:
                rospy.logerr("目标点%d 失败，停止导航" % i)
                return
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("✓✓✓ 所有目标点完成！任务结束！✓✓✓")
        rospy.loginfo("=" * 50)

def main():
    rospy.init_node('simple_goals_with_monitor')
    
    navigator = MultiGoalNavigator()
    
    # 等待接收到位置信息
    rospy.loginfo("等待位置信息...")
    while not navigator.position_updated and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    rospy.loginfo("开始导航任务！")
    navigator.navigate()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("导航被中断")

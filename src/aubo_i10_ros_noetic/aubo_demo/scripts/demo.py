#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander

class Demo:
    def __init__(self):

        # 初始化Python API 依赖的moveit_commanderC++系统，需放在前面
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('demo', anonymous=True)

        # 连接到想要控制的规划组xarm
        arm = MoveGroupCommander('arm')

        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置允许的最大速度
        arm.set_max_velocity_scaling_factor(0.8)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_joint_tolerance(0.001)


        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        joint_positions = [-0.5, 0.81, 0.21, -0.51, 0.41, -0.44]
        arm.set_joint_value_target(joint_positions)

        arm.go()
        rospy.sleep(1)

        # 干净地关闭moveit_commander并退出程序
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        Demo()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import Pose
from copy import deepcopy

class MoveItBeelineDemo:
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        #设置规划节点“moveot_beeline_demo”
        rospy.init_node('moveit_beeline_demo', anonymous=True)

        # 初始化需要控制的规划组
        arm = MoveGroupCommander('arm')

        # 允许重新规划
        arm.allow_replanning(True)
        arm.set_pose_reference_frame("base_link")

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.1)
        arm.set_goal_orientation_tolerance(0.1)

        # 设置允许的最大速度
        arm.set_max_velocity_scaling_factor(0.5)
        arm.set_max_acceleration_scaling_factor(0.5)

        end_effector_link=arm.get_end_effector_link()

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        # 设置三角形第一个顶点的位姿，让机械臂从初始状态运动到这个点
        start_pose=arm.get_current_pose(end_effector_link).pose

        # 初始化路点列表
        waypoints = []

        

        # 将start_pose加入路点列表
        waypoints.append(start_pose)

        # 设置第一个路径点为start_pose
        wpose = deepcopy(start_pose)

        # 设置第二个路径点, 把第二个路径点加入路点列表
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))

        # 设置第三个路径点,把第三个路径点加入路点列表
        wpose.position.x += 0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.2
        waypoints.append(deepcopy(wpose))

        fraction = 0.0   # 路径规划覆盖率
        maxtries = 100   # 最大尝试规划次数
        attempts = 0     # 已经尝试规划次数

        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()

        # 尝试规划一条笛卡尔路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path (
                                    waypoints,   # waypoint poses，路点列表，这里是4个点
                                    0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
                                    0,
                                    True)        # avoid_collisions，避障规划
            # 尝试次数累加
            attempts += 1
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

        rospy.sleep(1)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItBeelineDemo()
    except rospy.ROSInterruptException:
        pass

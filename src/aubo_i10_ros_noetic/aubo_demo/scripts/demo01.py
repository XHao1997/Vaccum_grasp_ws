#!/usr/bin/env python3
 
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose
 
 
# 初始化ROS节点
rospy.init_node('aubo_demo')  # 替换为你的节点名称
 
# 创建MoveIt Commander对象
robot = RobotCommander()
move_group = MoveGroupCommander('myrobot_arm')  # 替换为你的机械臂组名
 
# 获取当前的机器人状态
current_state = robot.get_current_state()
#rospy.loginfo("当前机械臂状态: %s", current_state)
 
# 创建PlanningSceneInterface对象
scene = PlanningSceneInterface()
rospy.sleep(2)  # 等待场景更新
 
# 设置目标位置坐标为(x, y, z)
target_position = Pose()
target_position.position.x = 0.0000  # 目标位置x坐标
target_position.position.y = 0.6916    # 目标位置y坐标
target_position.position.z = 0.6038    # 目标位置z坐标
target_position.orientation.w = 1.000000  # 确保方向正确
 
# 设置规划参数
 
move_group.set_num_planning_attempts(15)          # 增加规划尝试次数
move_group.set_planning_time(20)                  # 增加规划时间
move_group.set_goal_position_tolerance(0.05)      # 放宽位置容忍度
move_group.set_goal_orientation_tolerance(0.05)    # 放宽姿态容忍度
move_group.set_goal_tolerance(0.05)               # 设置整体容忍度
 
# 设置目标
move_group.set_pose_target(target_position)
#if move_group.check_collision():
#    rospy.logwarn("目标位置存在碰撞，请调整目标位置")
 
# 使用MoveGroupCommander检查当前状态的有效性（避免碰撞）
if move_group.get_current_state().joint_state:
    rospy.loginfo("当前状态有效，可以规划路径")
else:
    rospy.logerr("当前状态无效，发生碰撞或超出工作空间")
 
# 尝试路径规划
plan = move_group.plan()
#rospy.loginfo(f"规划路径: {plan}")
# 确认规划结果是否有效
if plan[0]:  # 检查plan是否是有效轨迹
    rospy.loginfo("规划成功，准备执行路径...")
 
    try:
        # 执行路径
        success = move_group.execute(plan[1], wait=True)
        if success:
            rospy.loginfo("成功将机械臂移动至目标位置")
        else:
            rospy.logwarn("执行失败，请检查轨迹或目标姿态")
    except Exception as e:
        rospy.logerr(f"执行时出现错误: {e}")
else:
    rospy.logerr("规划轨迹失败，检查障碍物信息或目标位置是否合理")
 
# 输出目标位置和当前机械臂末端状态，方便调试
current_pose = move_group.get_current_pose().pose
rospy.loginfo(f"目标位置: {target_position}")
rospy.loginfo(f"当前末端位置: {current_pose}")

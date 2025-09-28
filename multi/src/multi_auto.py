#!/usr/bin/env python3
import math
import rospy
from threading import Thread
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

# 定义全局变量
current_states = {}  # 存储多个无人机的状态，key为uav_namespace
current_poses = {}   # 存储多个无人机的位置，key为uav_namespace
uav_ready = {}       # 标记无人机是否准备好进行圆周运动
all_takeoff_complete = False  # 所有无人机是否完成起飞
n = input("请输入n的值：")      #飞机个数


def state_callback(msg, uav_namespace):
    global current_states
    current_states[uav_namespace] = msg

def pose_callback(msg, uav_namespace):
    global current_poses
    current_poses[uav_namespace] = msg

def init_ros_io(uav_namespace):
    # 带命名空间的回调函数
    rospy.Subscriber(f'/{uav_namespace}/mavros/state', 
                   State, 
                   lambda msg: state_callback(msg, uav_namespace))
    rospy.Subscriber(f'/{uav_namespace}/mavros/local_position/pose',
                   PoseStamped,
                   lambda msg: pose_callback(msg, uav_namespace))
    
    # 目标位置发布器
    pos_pub = rospy.Publisher(f'/{uav_namespace}/mavros/setpoint_position/local',
                             PoseStamped,
                             queue_size=10)
    
    # 服务客户端
    rospy.wait_for_service(f'/{uav_namespace}/mavros/cmd/arming')
    rospy.wait_for_service(f'/{uav_namespace}/mavros/set_mode')
    arm_client = rospy.ServiceProxy(f'/{uav_namespace}/mavros/cmd/arming', CommandBool)
    mode_client = rospy.ServiceProxy(f'/{uav_namespace}/mavros/set_mode', SetMode)

    return pos_pub, arm_client, mode_client


def wait_for_mavros_connection(uav_namespace):
    rospy.loginfo(f"{uav_namespace} 等待MAVROS连接...")
    while not rospy.is_shutdown() and (uav_namespace not in current_states or not current_states[uav_namespace].connected):
        rospy.sleep(0.5)
    if rospy.is_shutdown():
        rospy.logerr(f"{uav_namespace} 程序中断，未连接到MAVROS")
        exit(1)
    rospy.loginfo(f"{uav_namespace} MAVROS连接成功")


def send_init_setpoints(pos_pub, target_pose, duration=1.0, freq=20):
    rospy.loginfo(f"提前发送目标指令({duration}秒)...")
    rate = rospy.Rate(freq)
    for _ in range(int(duration * freq)):
        if rospy.is_shutdown():
            exit(1)
        target_pose.header.stamp = rospy.Time.now()
        pos_pub.publish(target_pose)
        rate.sleep()


def uav_takeoff(uav_namespace, target_alt=2.0):
    global uav_ready, all_takeoff_complete
    
    pos_pub, arm_client, mode_client = init_ros_io(uav_namespace)
    wait_for_mavros_connection(uav_namespace)
    
    target_pose = PoseStamped()
    target_pose.pose.position.z = target_alt
    target_pose.pose.orientation.w = 1.0

    send_init_setpoints(pos_pub, target_pose)
    
    # 解锁
    if not arm_client(True).success:
        rospy.logerr(f"{uav_namespace} 解锁失败")
        exit(1)
    rospy.loginfo(f"{uav_namespace} 解锁成功")

    # 切换至OFFBOARD模式
    rospy.loginfo(f"{uav_namespace} 切换至OFFBOARD模式...")
    for _ in range(5):
        if mode_client(0, "OFFBOARD").mode_sent:
            rospy.loginfo(f"{uav_namespace} OFFBOARD模式切换成功")
            break
        rospy.sleep(1)
    else:
        rospy.logerr(f"{uav_namespace} OFFBOARD切换失败")
        exit(1)

    # 等待到达目标高度
    rate = rospy.Rate(20)
    rospy.loginfo(f"{uav_namespace} 上升至{target_alt}米...")
    while not rospy.is_shutdown():
        target_pose.header.stamp = rospy.Time.now()
        pos_pub.publish(target_pose)

        if uav_namespace in current_poses and abs(current_poses[uav_namespace].pose.position.z - target_alt) < 0.1:
            rospy.loginfo(f"{uav_namespace} 已到达目标高度{target_alt}米")
            uav_ready[uav_namespace] = True
            break
            
        if current_states[uav_namespace].mode != "OFFBOARD":
            mode_client(0, "OFFBOARD")
        rate.sleep()

    # 等待所有无人机都完成起飞
    while not rospy.is_shutdown() and not all_takeoff_complete:
        rospy.sleep(0.1)

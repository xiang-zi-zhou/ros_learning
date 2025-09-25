#!/usr/bin/env python3

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped  # 用于发送位置指令
from std_msgs.msg import Header

# 全局变量
current_state = State()
target_pose = PoseStamped()  # 存储目标位置指令

def state_callback(msg):
    """更新无人机当前状态"""
    global current_state
    current_state = msg
###新增，融合circle
def pose_cb(msg):
    global current_pose
    current_pose = msg

def init_target_pose(home_altitude=0.0, target_altitude=2.0):
    """初始化目标位置（基于起飞点的相对位置）"""
    global target_pose
    # 位置参考：x=0, y=0 表示起飞点水平位置，z=目标高度
    target_pose.pose.position.x = 0.0
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = target_altitude  # 目标高度
    # 姿态：默认水平姿态（四元数表示，w=1表示无旋转）
    target_pose.pose.orientation.w = 1.0

def uav_takeoff_offboard(uav_namespace, target_altitude=2.0):
    """使用OFFBOARD模式控制无人机起飞"""
    global target_pose

    # 初始化节点
    if not rospy.core.is_initialized():
        rospy.init_node(f'{uav_namespace}_offboard_takeoff', anonymous=True)
    
    # 1. 订阅无人机状态，创建订阅器
    state_sub = rospy.Subscriber(
        f'/{uav_namespace}/mavros/state', 
        State, 
        state_callback
    )
    ###新增，融合circle
    pose_sub = rospy.Subscriber(
        'mavros/local_position/pose', 
        PoseStamped, 
        pose_cb,
        buff_size=1024*1024
    )

    # 2. 发布目标位置指令（OFFBOARD模式核心）创建发布器
    local_pos_pub = rospy.Publisher(
        f'/{uav_namespace}/mavros/setpoint_position/local', 
        PoseStamped, 
        queue_size=10
    )

    # # 3. 等待MAVROS连接
    # rospy.loginfo(f"等待 {uav_namespace} 连接到MAVROS...")
    # while not rospy.is_shutdown() and not current_state.connected:
    #     rospy.sleep(0.5)
    # if rospy.is_shutdown():
    #     return
    ######新增
    # 等待mavros服务
    print("等待mavros服务...")
    try:
        rospy.wait_for_service('mavros/cmd/arming', timeout=15)
        rospy.wait_for_service('mavros/set_mode', timeout=15)
    except rospy.ROSException:
        print("错误：无法连接到mavros服务！")
        print("请确保：1.PX4 SITL已启动 2.mavros已运行（udp连接）")
        return
    
    # 创建服务客户端
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
    print("成功连接到mavros服务")

    # 循环频率
    rate = rospy.Rate(20.0)
    
    # 等待与仿真器连接
    print("等待与PX4仿真器连接...")
    connect_timeout = rospy.Time.now() + rospy.Duration(10)
    while not rospy.is_shutdown():
        if current_state.connected:
            print("已与PX4仿真器建立连接！")
            print(f"当前初始状态 - 模式: {current_state.mode}, 解锁: {current_state.armed}")
            break
        if rospy.Time.now() > connect_timeout:
            print("连接超时！请检查mavros与PX4的UDP连接")
            return
        rospy.sleep(0.5)
        print("正在连接...")
    
    if rospy.is_shutdown():
        return
    

    ####新增结束

    # 4. 初始化目标位置（起飞点为原点，z=目标高度）
    init_target_pose(target_altitude=target_altitude)

    # 5. 提前发送目标指令（OFFBOARD模式要求先有指令再切模式）
    rospy.loginfo(f"提前发送目标位置指令（持续1秒）...")
    rate = rospy.Rate(20)  # 20Hz（PX4推荐不低于2Hz）
    for _ in range(20):  # 发送20次（1秒），确保指令被接收
        if rospy.is_shutdown():
            return
        # 必须更新header.stamp（否则指令可能被视为过期）
        target_pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(target_pose)
        rate.sleep()

    # 6. 服务代理（解锁、设置模式）
    arm_service = rospy.ServiceProxy(f'/{uav_namespace}/mavros/cmd/arming', CommandBool)
    set_mode_service = rospy.ServiceProxy(f'/{uav_namespace}/mavros/set_mode', SetMode)

    # 等待服务可用
    rospy.wait_for_service(f'/{uav_namespace}/mavros/cmd/arming')
    rospy.wait_for_service(f'/{uav_namespace}/mavros/set_mode')

    # 7. 解锁无人机
    rospy.loginfo(f"解锁 {uav_namespace}...")
    arm_response = arm_service(True)
    if not arm_response.success:
        rospy.logerr(f"{uav_namespace} 解锁失败！错误信息: {arm_response.result}")
        return
    rospy.loginfo(f"{uav_namespace} 解锁成功")

    # 8. 切换到OFFBOARD模式
    target_mode = "OFFBOARD"
    rospy.loginfo(f"设置 {uav_namespace} 为 {target_mode} 模式...")
    success = False
    for _ in range(5):  # 最多尝试5次
        set_mode_response = set_mode_service(base_mode=0, custom_mode=target_mode)
        if set_mode_response.mode_sent:
            rospy.loginfo(f"{uav_namespace} 模式切换指令已发送")
            success = True
            break
        rospy.logwarn(f"{uav_namespace} 模式切换尝试 {_+1} 失败，重试...")
        rospy.sleep(1)
    if not success:
        rospy.logerr(f"{uav_namespace} 切换到 {target_mode} 模式失败！")
        return

    # 9. 持续发送目标位置指令（核心！一旦停止发送会退出OFFBOARD模式）
    rospy.loginfo(f"{uav_namespace} 进入 {target_mode} 模式，正在上升至 {target_altitude} 米...")
    start_time = rospy.Time.now()
    while not rospy.is_shutdown():
        # 持续更新并发布目标位置
        target_pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(target_pose)

        # 检查是否到达目标高度（简化判断：持续发送指令10秒后视为到达）
        if (rospy.Time.now() - start_time).to_sec() > 10:
            rospy.loginfo(f"{uav_namespace} 已到达目标高度 {target_altitude} 米")
            break

        # 检查是否意外退出OFFBOARD模式
        if current_state.mode != target_mode:
            rospy.logwarn(f"警告：已退出 {target_mode} 模式，当前模式: {current_state.mode}")
            # 尝试重新切换模式
            set_mode_service(base_mode=0, custom_mode=target_mode)
        
        rate.sleep()

    ########新增，画圆


    # 10. 可选：切换到AUTO.LOITER模式悬停（防止退出OFFBOARD后失控）
    rospy.loginfo(f"切换 {uav_namespace} 到 AUTO.LOITER 模式悬停...")
    set_mode_service(base_mode=0, custom_mode="AUTO.LOITER")

if __name__ == '__main__':
    try:
        # 控制uav1使用OFFBOARD模式起飞到2米
        uav_takeoff_offboard('uav1', target_altitude=2.0)
        # 如需多机，添加：uav_takeoff_offboard('uav2', 2.0)
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")

#!/usr/bin/env python3
import math
import rospy
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler


# 全局变量
current_state = State()
target_pose = PoseStamped()  # 存储目标位置指令
current_pose = PoseStamped()  # 存储无人机当前位置

def state_callback(msg):
    """更新无人机当前状态"""
    global current_state
    current_state = msg

def pose_callback(msg):
    """更新无人机当前位置"""
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

    # 1. 订阅无人机状态和位置
    state_sub = rospy.Subscriber(
        f'/{uav_namespace}/mavros/state', 
        State, 
        state_callback
    )
    pose_sub = rospy.Subscriber(
        f'/{uav_namespace}/mavros/local_position/pose', 
        PoseStamped, 
        pose_callback
    )

    # 2. 发布目标位置指令（OFFBOARD模式核心）
    local_pos_pub = rospy.Publisher(
        f'/{uav_namespace}/mavros/setpoint_position/local', 
        PoseStamped, 
        queue_size=10
    )

    # 3. 等待MAVROS连接
    rospy.loginfo(f"等待 {uav_namespace} 连接到MAVROS...")
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.sleep(0.5)
    if rospy.is_shutdown():
        return

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

    # 9. 持续发送目标位置指令，直到到达目标高度
    rospy.loginfo(f"{uav_namespace} 进入 {target_mode} 模式，正在上升至 {target_altitude} 米...")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # 持续更新并发布目标位置
        target_pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(target_pose)

        # 基于实际高度判断是否到达目标（允许±0.1米误差）
        if abs(current_pose.pose.position.z - target_altitude) < 0.1:
            rospy.loginfo(f"{uav_namespace} 已到达目标高度 {target_altitude} 米")
            break

        # 检查是否意外退出OFFBOARD模式
        if current_state.mode != target_mode:
            rospy.logwarn(f"警告：已退出 {target_mode} 模式，当前模式: {current_state.mode}")
            # 尝试重新切换模式
            set_mode_service(base_mode=0, custom_mode=target_mode)
        
        rate.sleep()

def uav_circle(uav_namespace):
    print("=== 启动PX4 SITL圆形飞行脚本 ===")
    
    # 等待current_pose初始化（避免NoneType错误）
    while not rospy.is_shutdown() and current_pose.header.frame_id == "":
        rospy.loginfo("等待获取无人机当前位置...")
        rospy.sleep(0.5)
    
    # 创建订阅器
    state_sub = rospy.Subscriber(
        f'/{uav_namespace}/mavros/state', 
        State, 
        state_callback,
        buff_size=1024*1024
    )
    pose_sub = rospy.Subscriber(
        f'/{uav_namespace}/mavros/local_position/pose', 
        PoseStamped, 
        pose_callback,
        buff_size=1024*1024
    )
    
    # 创建发布器
    local_pos_pub = rospy.Publisher(
        f'/{uav_namespace}/mavros/setpoint_position/local', 
        PoseStamped, 
        queue_size=10
    )
    
    # 等待mavros服务
    print("等待mavros服务...")
    try:
        rospy.wait_for_service(f'/{uav_namespace}/mavros/cmd/arming', timeout=15)
        rospy.wait_for_service(f'/{uav_namespace}/mavros/set_mode', timeout=15)
    except rospy.ROSException:
        print("错误:无法连接到mavros服务！")
        print("请确保：1.PX4 SITL已启动 2.mavros已运行（udp连接）")
        return
    
    # 创建服务客户端
    arming_client = rospy.ServiceProxy(f'/{uav_namespace}/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy(f'/{uav_namespace}/mavros/set_mode', SetMode)
    print("成功连接到mavros服务")
    
    # 循环频率
    rate = rospy.Rate(20.0)
    
    # 发送初始设置点
    print("发送初始设置点...")
    for i in range(100):
        if rospy.is_shutdown():
            return
        if current_pose.header.frame_id == "":
            current_pose.header.frame_id = "map"
        current_pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(current_pose)
        rospy.sleep(0.05)
        if i % 20 == 0:
            print(f"发送初始点: {i+1}/100")
    
    # 准备模式请求（确保处于OFFBOARD模式）
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = "OFFBOARD"
    last_request = rospy.Time.now()
    timeout = rospy.Time.now() + rospy.Duration(15)
    
    # 确保无人机处于OFFBOARD模式
    print("确认无人机处于OFFBOARD模式...")
    while not rospy.is_shutdown():
        if current_state.mode == "OFFBOARD":
            print("✅ 已确认：无人机处于OFFBOARD模式")
            break
        
        if rospy.Time.now() > timeout:
            print("⚠️  超时警告：未能进入OFFBOARD模式")
            print(f"当前状态 - 模式: {current_state.mode}")
            return
        
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(2.0)):
            print(f"尝试切换模式（当前模式：{current_state.mode}）...")
            try:
                response = set_mode_client(0, offb_set_mode.custom_mode)
                if response.mode_sent:
                    print("模式切换请求已发送")
            except Exception as e:
                print(f"模式切换服务调用失败: {str(e)}")
            last_request = rospy.Time.now()
        
        local_pos_pub.publish(current_pose)
        rate.sleep()
    
    # 飞行参数
    radius = 5.0
    altitude = 3.0  # 比起飞高度高1米
    angular_vel = 0.5
    start_time = rospy.Time.now()
    
    # 计算圆心位置
    if abs(current_pose.pose.position.x) < 0.1 and abs(current_pose.pose.position.y) < 0.1:
        center_x = -radius  # 从原点起飞时，圆心在左侧radius处
    else:
        center_x = current_pose.pose.position.x - radius
    center = current_pose.pose.position
    center.x = center_x
    
    print(f"\n🚀 开始圆形飞行任务")
    print(f"参数 - 半径: {radius}m, 高度: {altitude}m, 角速度: {angular_vel}rad/s")
    print(f"圆心位置: X={center.x:.2f}, Y={center.y:.2f}")
    
    # 主飞行循环
    pose = PoseStamped()
    pose.header.frame_id = "map"
    
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        angle = angular_vel * elapsed
        
        # 计算圆形轨迹位置
        pose.pose.position.x = center.x + radius * math.cos(angle)
        pose.pose.position.y = center.y + radius * math.sin(angle)
        pose.pose.position.z = altitude
        
        # 朝向圆心
        yaw = math.atan2(center.y - pose.pose.position.y, center.x - pose.pose.position.x)
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        
        # 每30度打印一次状态
        if int(math.degrees(angle)) % 30 == 0 and angle % (math.pi/3) < 0.1:
            print(f"飞行状态 - 角度: {math.degrees(angle):.0f}°, 位置: X={pose.pose.position.x:.2f}, Y={pose.pose.position.y:.2f}")
        
        # 完成一圈后返回起点
        if angle >= 2 * math.pi:
            print("\n✅ 已完成一圈飞行，返回起点")
            # 发送起点位置
            pose.pose.position.x = current_pose.pose.position.x
            pose.pose.position.y = current_pose.pose.position.y
            pose.pose.position.z = altitude
            pose.header.stamp = rospy.Time.now()
            for _ in range(50):  # 连续发送多次确保到达
                local_pos_pub.publish(pose)
                rate.sleep()
            rospy.sleep(3)
            break
        
        rate.sleep()
    
    print("\n🎯 飞行任务完成！")

if __name__ == '__main__':
    try:
        # 全局唯一的节点初始化
        rospy.init_node('uav_multi_circle_node', anonymous=True)
        # 控制uav1使用OFFBOARD模式起飞到2米
        uav_takeoff_offboard('uav1', target_altitude=2.0)
        # 执行圆形飞行任务
        uav_circle('uav1')
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")

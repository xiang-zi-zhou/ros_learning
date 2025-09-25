#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, SetModeRequest
from tf.transformations import quaternion_from_euler

# 全局变量
current_state = State()
current_pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global current_pose
    current_pose = msg

def main():
    print("=== 启动PX4 SITL圆形飞行脚本 ===")
    
    # 初始化节点
    rospy.init_node('sitl_circle_fly')
    print("ROS节点初始化完成")
    
    # 创建订阅器
    state_sub = rospy.Subscriber(
        'mavros/state', 
        State, 
        state_cb,
        buff_size=1024*1024
    )
    pose_sub = rospy.Subscriber(
        'mavros/local_position/pose', 
        PoseStamped, 
        pose_cb,
        buff_size=1024*1024
    )
    
    # 创建发布器
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
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
    
    # 准备模式和解锁请求
    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = "OFFBOARD"
    arm_cmd = CommandBool()
    arm_cmd.value = True
    last_request = rospy.Time.now()
    timeout = rospy.Time.now() + rospy.Duration(15)
    
    # 模式切换和解锁逻辑
    print("尝试切换到OFFBOARD模式并解锁...")
    while not rospy.is_shutdown():
        if current_state.mode == "OFFBOARD" and current_state.armed:
            print("✅ 已确认：成功进入OFFBOARD模式并解锁！")
            print(f"当前状态 - 模式: {current_state.mode}, 解锁: {current_state.armed}")
            break
        
        if rospy.Time.now() > timeout:
            print("⚠️  超时警告：未能完成模式切换/解锁")
            print(f"当前状态 - 模式: {current_state.mode}, 解锁: {current_state.armed}")
            print("建议：重启PX4 SITL和mavros后重试")
            return
        
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(2.0)):
            print(f"尝试切换模式（当前模式：{current_state.mode}）...")
            try:
                response = set_mode_client(0, offb_set_mode.custom_mode)
                if response.mode_sent:
                    print("模式切换请求已发送")
                else:
                    print("模式切换请求发送失败")
            except Exception as e:
                print(f"模式切换服务调用失败: {str(e)}")
            last_request = rospy.Time.now()
        
        if not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(2.0)):
            print("尝试解锁...")
            try:
                response = arming_client(arm_cmd.value)
                if response.success:
                    print("解锁请求已发送")
                else:
                    print("解锁请求发送失败")
            except Exception as e:
                print(f"解锁服务调用失败: {str(e)}")
            last_request = rospy.Time.now()
        
        local_pos_pub.publish(current_pose)
        rate.sleep()
    
    # 飞行参数
    radius = 5.0
    altitude = 3.0
    angular_vel = 0.5
    start_time = rospy.Time.now()
    
    if current_pose.pose.position.x == 0 and current_pose.pose.position.y == 0:
        center_x = -radius
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
        
        pose.pose.position.x = center.x + radius * math.cos(angle)
        pose.pose.position.y = center.y + radius * math.sin(angle)
        pose.pose.position.z = altitude
        
        yaw = math.atan2(center.y - pose.pose.position.y, center.x - pose.pose.position.x)
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        
        pose.header.stamp = rospy.Time.now()
        local_pos_pub.publish(pose)
        
        if int(math.degrees(angle)) % 30 == 0 and angle % (math.pi/3) < 0.1:
            print(f"飞行状态 - 角度: {math.degrees(angle):.0f}°, 位置: X={pose.pose.position.x:.2f}, Y={pose.pose.position.y:.2f}")
        
        if angle >= 2 * math.pi:
            print("\n✅ 已完成一圈飞行，返回起点")
            pose.pose.position.x = current_pose.pose.position.x
            pose.pose.position.y = current_pose.pose.position.y
            pose.pose.position.z = altitude
            pose.header.stamp = rospy.Time.now()
            local_pos_pub.publish(pose)
            rospy.sleep(3)
            break
        
        rate.sleep()
    
    print("\n🎯 飞行任务完成！")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("\n程序被手动中断")
    except Exception as e:
        print(f"\n发生错误: {str(e)}")
    print("=== 脚本结束 ===")

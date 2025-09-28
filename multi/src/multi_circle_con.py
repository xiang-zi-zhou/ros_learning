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

def uav_circle(uav_namespace, radius=5.0, target_alt=3.0, angular_vel=0.1):
    pos_pub, _, _ = init_ros_io(uav_namespace)

    rospy.loginfo(f"{uav_namespace} 等待无人机初始化...")
    while not rospy.is_shutdown() and (uav_namespace not in current_poses or current_poses[uav_namespace].header.frame_id == ""):
        rospy.sleep(0.5)
    
    # 计算圆心
    center = current_poses[uav_namespace].pose.position
    center.x -= radius
    
    # 等待起飞完成信号
    global all_takeoff_complete
    while not rospy.is_shutdown() and not all_takeoff_complete:
        rospy.sleep(0.1)

    # 绕圈主逻辑
    rate = rospy.Rate(50)
    start_time = rospy.Time.now()
    rospy.loginfo(f"{uav_namespace} 开始绕圈：半径{radius}m，高度{target_alt}m，圆心({center.x:.2f},{center.y:.2f})")

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.z = target_alt

    while not rospy.is_shutdown():
        # 计算当前角度
        elapsed = (rospy.Time.now() - start_time).to_sec()
        angle = angular_vel * elapsed

        # 计算圆形轨迹位置
        pose.pose.position.x = center.x + radius * math.cos(angle)
        pose.pose.position.y = center.y + radius * math.sin(angle)

        # 朝向圆心
        yaw = math.atan2(center.y - pose.pose.position.y, center.x - pose.pose.position.x)
        q = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion()
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]
        pose.pose.orientation = orientation

        # 发布指令
        pose.header.stamp = rospy.Time.now()
        pos_pub.publish(pose)

        # # 每60度打印一次状态
        # if int(math.degrees(angle)) % 60 == 0 and angle % (math.pi/3) < 0.1:
        #     rospy.loginfo(f"{uav_namespace} 绕圈进度：{math.degrees(angle):.0f}° | 位置(X,Y): ({pose.pose.position.x:.2f},{pose.pose.position.y:.2f})")
        
        # 完成一圈+0.5*Pi后返回起点并退出
        if angle >= 2.5 * math.pi:
            rospy.loginfo(f"{uav_namespace} 完成一圈，返回起点")
            # 发送起点位置（连续50次确保稳定）
            pose.pose.position = current_poses[uav_namespace].pose.position
            rospy.loginfo(f"{uav_namespace} 起点{current_poses[uav_namespace].pose.position}")
            for _ in range(50):
                pos_pub.publish(pose)
                rate.sleep()
            break
        rate.sleep()
    rospy.loginfo(f"{uav_namespace} 绕圈任务完成!")


def check_all_takeoff_complete(uav_list):
    global all_takeoff_complete, uav_ready
    rospy.loginfo("等待所有无人机完成起飞...")
    while not rospy.is_shutdown():
        # 检查所有无人机是否都已准备好
        if all(uav in uav_ready and uav_ready[uav] for uav in uav_list):
            all_takeoff_complete = True
            rospy.loginfo("所有无人机已完成起飞，开始同步执行圆周运动!")
            break
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        # 初始化节点
        rospy.init_node('uav_multi_sync_control', anonymous=True)
        
        # 定义无人机列表及参数
        uavs = [
            {'namespace': 'uav0', 'takeoff_alt': 2.0, 'circle_radius': 5.0, 'circle_alt': 3.0},
            {'namespace': 'uav1', 'takeoff_alt': 5.0, 'circle_radius': 3.0, 'circle_alt': 5.0}
        ]
        #### 列表推导式（List Comprehension）：从 uavs 列表中，批量提取每架无人机的 namespace（命名空间），生成一个新的列表
        uav_namespaces = [uav['namespace'] for uav in uavs]
        
        # 启动线程监控所有无人机是否完成起飞，专门监控所有无人机起飞状态的线程
        Thread(target=check_all_takeoff_complete, args=(uav_namespaces,), daemon=True).start()
        
        # 为每个无人机启动起飞线程
        takeoff_threads = []###创建空列表
        for uav in uavs:
            thread = Thread(
                target=uav_takeoff,
                args=(uav['namespace'], uav['takeoff_alt']),
                daemon=True
            )
            takeoff_threads.append(thread)##########append() 将一个元素（这里是线程对象 thread）添加到列表的末尾【填满空列表】
            thread.start()
        
        # 等待所有起飞线程完成（实际是等待同步信号）
        for thread in takeoff_threads:
            thread.join()
        
        # 为每个无人机启动圆周运动线程
        circle_threads = []
        for uav in uavs:
            thread = Thread(
                target=uav_circle,
                args=(uav['namespace'], uav['circle_radius'], uav['circle_alt']),
                daemon=True
            )
            circle_threads.append(thread)
            thread.start()
        
        # 等待所有圆周运动完成
        for thread in circle_threads:
            thread.join()
            
        rospy.loginfo("所有任务完成!")

    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"错误：{str(e)}")

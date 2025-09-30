#!/usr/bin/env python3
import math
import rospy
from threading import Thread
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

#定义全局变量
current_states = {}  # 存储多个无人机的状态，key为uav_namespace
current_poses = {}   # 存储多个无人机的位置，key为uav_namespace
uav_ready = {}       # 标记无人机是否准备好进行圆周运动
all_takeoff_complete = False  # 所有无人机是否完成起飞
radius = 10
target_alt = 5
center_x = 10
center_y = 10
dis_r = 4


# 使用input()函数获取用户输入，并用int()转换为整数类型
def get_num():
    user_input = input("请输入一个整数: ")
    # 将输入转换为整数并赋值给变量
    global number
    number = int(user_input)
    # 显示结果
    print(f"你输入的整数是: {number}")
    print(f"变量number的值已被设置为: {number}")
    # if ValueError:
    #     print("错误: 你输入的不是一个有效的整数，请重试。")
    return number


def generate_uav_positions(n):
    """
    生成在平行于z轴的圆上呈正多边形分布的无人机目标坐标
    
    参数:
        n: 无人机数量，1~9之间的整数
        
    返回:
        dict: 包含每个无人机名称及其坐标的字典，格式为{uav名称: (x, y, z)}
    """
    # 圆1的参数（平面平行于z轴）
    center_x, center_y, center_z = 10.0, 0.0, 5.0  # 圆心坐标
    radius = 4.0  # 半径
    
    # 验证输入n的有效性
    if not isinstance(n, int) or n < 1 or n > 9:
        raise ValueError("n必须是1到9之间的整数")
    
    uav_positions = {}
    
    if n == 1:
        # 特殊情况：n=1时放在圆的最顶点（z轴正方向）
        uav_positions["uav0"] = (center_x, center_y, center_z + radius)
    else:
        # 计算正多边形的每个顶点角度（弧度）
        # 对于平行于z轴的圆，我们在y-z平面内计算角度
        # 确保关于过圆心且与z轴平行的直线（x=10, y=0）对称
        for i in range(n):
            angle = 2 * math.pi * i / n
            # 在y-z平面内计算坐标（x坐标固定为圆心x值）
            y = center_y + radius * math.sin(angle)
            z = center_z + radius * math.cos(angle)
            x = center_x  # x坐标与圆心相同，保持在平行于z轴的平面内
            
            uav_name = f"uav{i}"
            uav_positions[uav_name] = (round(x, 4), round(y, 4), round(z, 4))
    
    return uav_positions

# # 示例用法
# if __name__ == "__main__":
#     # 测试不同n值的情况
#     for n in [1, 2, 3, 4, 6]:
#         print(f"n={n}时的无人机位置:")
#         positions = generate_uav_positions(n)
#         for uav, pos in positions.items():
#             print(f"  {uav}: {pos}")
#         print()


#  ####实现位置控制
# def set_drone_position(uav_namespace,x, y, z):
#     # # 初始化节点
#     # rospy.init_node('drone_position_controller', anonymous=True)
    
#     # 创建发布者，发布到位置控制话题
#     setpoint_pub = rospy.Publisher(f'/{uav_namespace}/mavros/setpoint_position/local', 
#                                   PoseStamped, queue_size=10)
    
#     # 设置循环频率
#     rate = rospy.Rate(10)  # 10Hz
    
#     # 等待节点初始化完成
#     while not rospy.is_shutdown():
#         # 创建位置消息
#         pose = PoseStamped()
#         pose.header.stamp = rospy.Time.now()
#         pose.header.frame_id = "map"  # 使用地图坐标系
        
#         # 设置目标位置 (x, y, z)，单位：米
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.position.z = z
        
#         # 设置姿态（默认水平）
#         pose.pose.orientation.w = 1.0  # 四元数，这里表示水平姿态
        
#         # 发布位置指令
#         setpoint_pub.publish(pose)
        
#         # 检查是否到达目标（简单判断，实际应用需更复杂逻辑）
#         rospy.loginfo(f"发送目标位置: x={x}, y={y}, z={z}")
        
#         rate.sleep()

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

def uav_takeoff(uav_namespace, target_x, target_y, target_z):
    global uav_ready, all_takeoff_complete
    
    pos_pub, arm_client, mode_client = init_ros_io(uav_namespace)
    wait_for_mavros_connection(uav_namespace)
    
    target_pose = PoseStamped()
    # 使用从字典获取的目标坐标
    # target_pose.header.frame_id = f"map/{uav_namespace}"  # 标记所属无人机
    target_pose.header.frame_id = "map" 
    target_pose.pose.position.x = target_x
    target_pose.pose.position.y = target_y
    target_pose.pose.position.z = target_z
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

    # 等待到达目标位置
    rate = rospy.Rate(20)
    rospy.loginfo(f"{uav_namespace} 前往目标位置 ({target_x}, {target_y}, {target_z})...")
    while not rospy.is_shutdown():
        target_pose.header.stamp = rospy.Time.now()
        pos_pub.publish(target_pose)

        if uav_namespace in current_poses:
            current_x = current_poses[uav_namespace].pose.position.x
            current_y = current_poses[uav_namespace].pose.position.y
            current_z = current_poses[uav_namespace].pose.position.z
            
            # 检查是否到达目标位置（允许一定误差）
            if (abs(current_x - target_x) < 0.05 and 
                abs(current_y - target_y) < 0.05 and 
                abs(current_z - target_z) < 0.05):
                rospy.loginfo(f"{uav_namespace} 已到达目标位置")
                uav_ready[uav_namespace] = True
                break
                
        if current_states[uav_namespace].mode != "OFFBOARD":
            mode_client(0, "OFFBOARD")
        rate.sleep()

    # 等待所有无人机都完成起飞
    while not rospy.is_shutdown() and not all_takeoff_complete:
        rospy.sleep(5)

def uav_circle(uav_namespace, radius=10.0, target_alt=5.0, angular_vel=0.1):
    pos_pub, _, _ = init_ros_io(uav_namespace)

    rospy.loginfo(f"{uav_namespace} 等待无人机初始化...")
    while not rospy.is_shutdown() and (uav_namespace not in current_poses or current_poses[uav_namespace].header.frame_id == ""):
        rospy.sleep(0.5)
    
    # 计算圆心（基于起飞后的位置）
    # center = current_poses[uav_namespace].pose.position
    # center.x -= radius
    center = type('', (), {})()  # 创建一个空对象存储圆心坐标
    center.x = 10
    center.y = 10
    center.z = 5.0  

    # 反算初始角度θ（弧度制）
    Px = current_poses[uav_namespace].pose.position.x
    Py = current_poses[uav_namespace].pose.position.y
    delta_x = Px - center.x
    delta_y = Py - center.y
    theta = math.atan2(delta_y, delta_x) 
    
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
        angle = angular_vel * elapsed + theta

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

        # 完成一圈后返回起点并退出
        if angle - theta >= 2 * math.pi:
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
            rospy.loginfo("所有无人机起飞完成，当前map坐标：")
            for uav in uav_list:
                x = current_poses[uav].pose.position.x
                y = current_poses[uav].pose.position.y
                z = current_poses[uav].pose.position.z
                rospy.loginfo(f"{uav}: (x={x:.2f}, y={y:.2f}, z={z:.2f})")
            all_takeoff_complete = True
            rospy.loginfo("所有无人机已完成起飞，开始同步执行圆周运动!")
            break
        rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        # 初始化节点
        rospy.init_node('uav_multi_sync_control', anonymous=True)
        numb=get_num()
        
        # 3. 生成无人机坐标字典（使用提供的generate_uav_positions函数）
        uav_coords = generate_uav_positions(numb)
        rospy.loginfo(f"成功生成{numb}架无人机的坐标：\n{uav_coords}")
        
        # 4. 构建无人机控制参数列表（整合坐标与运动参数）
        uavs = []
        for uav_name, (x, y, z) in uav_coords.items():
            uavs.append({
                "namespace": uav_name,       # 无人机命名空间（如uav0）
                "takeoff_x": x,              # 起飞x坐标（来自生成的字典）
                "takeoff_y": y,              # 起飞y坐标（来自生成的字典）
                "takeoff_z": z,              # 起飞z坐标（来自生成的字典）
                "circle_radius": 10.0 - y,        # 绕圈半径（可根据需求调整）
                "circle_angular_vel": 0.1    # 绕圈角速度（单位：rad/s）
            })
        uav_namespaces = [uav["namespace"] for uav in uavs]  # 提取所有无人机命名空间
        
        # 5. 启动起飞监控线程（检查所有无人机是否完成起飞）
        # 该线程会在所有无人机起飞后设置all_takeoff_complete=True，触发同步绕圈
        Thread(
            target=check_all_takeoff_complete,
            args=(uav_namespaces,),
            daemon=True  # 主线程退出时自动结束
        ).start()
        
        # 6. 为每架无人机启动起飞线程
        takeoff_threads = []
        for uav in uavs:
            # 创建起飞线程，传入无人机命名空间和目标坐标
            thread = Thread(
                target=uav_takeoff,
                args=(
                    uav["namespace"],
                    uav["takeoff_x"],
                    uav["takeoff_y"],
                    uav["takeoff_z"]
                ),
                daemon=True
            )
            takeoff_threads.append(thread)
            thread.start()
            rospy.loginfo(f"已启动{uav['namespace']}的起飞线程")
            print(thread)#######打印
        
        # 7. 等待所有起飞线程完成（确保所有无人机到达目标位置）
        for thread in takeoff_threads:
            thread.join()  # 阻塞等待线程结束
        
        # 8. 为每架无人机启动绕圈线程（所有无人机同步开始）
        circle_threads = []
        for uav in uavs:
            # 创建绕圈线程，传入无人机命名空间和绕圈参数
            thread = Thread(
                target=uav_circle,
                args=(
                    uav["namespace"],
                    uav["circle_radius"],
                    uav["takeoff_z"]
                ),
                daemon=True
            )
            circle_threads.append(thread)
            thread.start()
            rospy.loginfo(f"已启动{uav['namespace']}的绕圈线程")
        
        # 9. 等待所有绕圈线程完成（所有无人机完成绕圈任务）
        for thread in circle_threads:
            thread.join()
        
        # 10. 所有任务完成
        rospy.loginfo("="*50)
        rospy.loginfo(f"所有{numb}架无人机已完成起飞和绕圈任务！")
        rospy.loginfo("="*50)

    except rospy.ROSInterruptException:
        rospy.loginfo("程序被ROS中断（如Ctrl+C）")
    except ValueError as ve:
        rospy.logerr(f"参数错误：{str(ve)}")  # 捕获generate_uav_positions的参数异常
    except Exception as e:
        rospy.logerr(f"程序发生错误：{str(e)}")

#!/usr/bin/env python3
import math
import rospy
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

#定义全局变量
current_state = State()
current_pose = PoseStamped()

def state_callback(msg):
    global current_state
    current_state = msg

def pose_callback(msg):
    global current_pose
    current_pose = msg


def init_ros_io(uav_namespace):
    rospy.Subscriber(f'/{uav_namespace}/mavros/state',State,state_callback)
    rospy.Subscriber(f'/{uav_namespace}/mavros/local_position/pose',PoseStamped,pose_callback)
    #目标任务位置
    pos_pub = rospy.Publisher(f'/{uav_namespace}/mavros/setpoint_position/local',PoseStamped,queue_size=10)
    #创建客户端和服务端，发送请求“解锁”与“模式设置”
    rospy.wait_for_service(f'/{uav_namespace}/mavros/cmd/arming')
    rospy.wait_for_service(f'/{uav_namespace}/mavros/set_mode')
    arm_client = rospy.ServiceProxy(f'/{uav_namespace}/mavros/cmd/arming',CommandBool)
    mode_client = rospy.ServiceProxy(f'/{uav_namespace}/mavros/set_mode',SetMode)

    return pos_pub,arm_client,mode_client

def wait_for_mavros_connection():
    rospy.loginfo("等待MAVROS连接...")
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.sleep(0.5)
    if rospy.is_shutdown():
        rospy.logerr("程序中断，未连接到MAVROS")
        exit(1)
    rospy.loginfo("MAVROS连接成功")

"""【抽象初始指令发送】合并起飞和绕圈前的“提前发指令”逻辑"""
###不懂
def send_init_setpoints(pos_pub, target_pose, duration=1.0, freq=20):
    rospy.loginfo(f"提前发送目标指令({duration}秒)...")
    rate = rospy.Rate(freq)
    for _ in range(int(duration * freq)):
        if rospy.is_shutdown():
            exit(1)
        target_pose.header.stamp = rospy.Time.now()
        pos_pub.publish(target_pose)
        rate.sleep()

def uav_takeoff(uav_namespace, target_alt = 2.0):
    pos_pub, arm_client, mode_client = init_ros_io(uav_namespace)

    wait_for_mavros_connection()
    target_pose = PoseStamped()
    target_pose.pose.position.z = target_alt
    target_pose.pose.orientation.w = 1.0

    send_init_setpoints(pos_pub, target_pose)
    if not arm_client(True).success:
        rospy.logerr(f"{uav_namespace}解锁失败")
        exit(1)
    rospy.loginfo(f"{uav_namespace}解锁成功")

    rospy.loginfo("切换至OFFBOARD模式...")
    for _ in range(5):
        if mode_client(0,"OFFBOARD").mode_sent:
            rospy.loginfo("OFFBOARD模式切换成功")
            break
        rospy.sleep(1)
    else:
        rospy.logerr("OFFBOARD切换失败")
        exit(1)

    ##等待到达目标高度
    rate = rospy.Rate(20)
    rospy.loginfo(f"上升至{target_alt}米...")
    while not rospy.is_shutdown():
        target_pose.header.stamp = rospy.Time.now()
        pos_pub.publish(target_pose)

        ##
        if abs(current_pose.pose.position.z-target_alt) < 0.1:
            rospy.loginfo(f"已到达目标高度{target_alt}米")
            break
        if current_state.mode != "OFFBOARD":
            mode_client(0,"OFFBOARD")
        rate.sleep()

def uav_cicle(uav_namespace, radius=5.0, target_alt=3.0,angular_vel=0.5):
    pos_pub, _, _ = init_ros_io(uav_namespace)

    rospy.loginfo("等待无人机初始化...")
    while not rospy.is_shutdown() and current_pose.header.frame_id == "":
        rospy.sleep(0.5)
    ######计算圆心
    center = current_pose.pose.position
    center.x -= radius
    #####绕圈主逻辑
    rate = rospy.Rate(50)
    start_time = rospy.Time.now()
    rospy.loginfo(f"开始绕圈：半径{radius}m，高度{target_alt}m，圆心({center.x:.2f},{center.y:.2f})")

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.z = target_alt######固定高度

    while not rospy.is_shutdown():
        ####计算当前角度（基于时间的角速度积分）
        elapsed = (rospy.Time.now() - start_time).to_sec()
        angle = angular_vel * elapsed

        ##计算圆形轨迹位置
        pose.pose.position.x = center.x + radius * math.cos(angle)
        pose.pose.position.y = center.y + radius * math.sin(angle)

        ####朝向圆心（四元数转化）
        yaw = math.atan2(center.y - pose.pose.position.y, center.x - pose.pose.position.x)
        q = quaternion_from_euler(0, 0, yaw)
        orientation = Quaternion()
        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]
        pose.pose.orientation = orientation

        # pose.pose.orientation = [q[0], q[1], q[2], q[3]]


        ###发布指令
        pose.header.stamp = rospy.Time.now()
        pos_pub.publish(pose)

        # 每60度打印一次状态（减少冗余输出）
        if int(math.degrees(angle)) % 60 == 0 and angle % (math.pi/3) < 0.1:
            rospy.loginfo(f"绕圈进度：{math.degrees(angle):.0f}° | 位置(X,Y): ({pose.pose.position.x:.2f},{pose.pose.position.y:.2f})")
        
        #####完成一圈后返回起点并退出
        if angle >= 2.5 * math.pi:
            rospy.loginfo("完成一圈，返回起点")
            ##发送起点位置（连续50次确保稳定）
            pose.pose.position = current_pose.pose.position
            rospy.loginfo(f"起点{current_pose.pose.position}")
            for _ in range(50):
                pos_pub.publish(pose)
                rate.sleep()
            break
        rate.sleep()
    rospy.loginfo("绕圈任务完成!")



if __name__ == '__main__':
    try:
        #####初始化节点
        rospy.init_node('uav_takeoff_circle_node',anonymous=True)
        uav_takeoff(uav_namespace='uav0',target_alt=2.0)
        uav_cicle(uav_namespace='uav0',radius=5.0,target_alt=3.0)
        rospy.loginfo("换一个飞机")
        uav_takeoff(uav_namespace='uav1',target_alt=5.0)
        uav_cicle(uav_namespace='uav1',radius=3.0,target_alt=5.0)
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断")
    except Exception as e:
        rospy.logerr(f"错误：{str(e)}")

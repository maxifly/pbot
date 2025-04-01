#!/usr/bin/env python
import math

import rospy
from nav_msgs.msg import Odometry
from rospy import Time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import tf
import threading

ANGLE_LIMIT = 2 * math.pi

# TODO Поменять
wheel_radius = rospy.get_param('~wheel_radius', 0.1)  # Радиус колеса в метрах
wheel_separation_length = rospy.get_param('~wheel_separation_length',
                                          0.5)  # Расстояние между левыми и правыми колёсами в метрах
wheel_separation_width = rospy.get_param('~wheel_separation_width',
                                         0.3)  # Расстояние между передними и задними колёсами в метрах


class AllJointsState:
    def __init__(self):
        self.current_right_rotation_speed = 0.
        self.current_left_rotation_speed = 0.


class AllStateContext:
    def __init__(self):
        joint_state = JointState()
        joint_state.name = ['pbot_lf_w_to_base', 'pbot_lr_w_to_base', 'pbot_rf_w_to_base', 'pbot_rr_w_to_base']
        joint_state.position = [0.0, 0.0, 0.0, 0.0]
        joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
        joint_state.effort = [0.0, 0.0, 0.0, 0.0]

        self.wheel_joint = joint_state
        self.prev_time = rospy.Time.now()
        self.left_position = 0.0
        self.right_position = 0.0

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.v_left = 0.0
        self.v_right = 0.0


def create_wheel_joint_state(all_joints: AllJointsState, context: AllStateContext, current_time: Time) -> JointState:
    delta_t = (current_time - context.prev_time).to_sec()

    right_position_delta = all_joints.current_right_rotation_speed * delta_t
    left_position_delta = all_joints.current_left_rotation_speed * delta_t

    context.left_position += left_position_delta
    context.right_position += right_position_delta
    context.left_position %= ANGLE_LIMIT
    context.right_position %= ANGLE_LIMIT

    # Обновляем положение колёс
    context.wheel_joint.position[0] = context.left_position
    context.wheel_joint.position[1] = context.left_position
    context.wheel_joint.position[2] = context.right_position
    context.wheel_joint.position[3] = context.right_position

    # Обновляем скорость
    context.wheel_joint.velocity = [all_joints.current_left_rotation_speed, all_joints.current_left_rotation_speed,
                                    all_joints.current_right_rotation_speed, all_joints.current_right_rotation_speed]

    # Обновляем время
    context.wheel_joint.header.stamp = current_time

    return context.wheel_joint


def odometry_state(all_joints: AllJointsState, context: AllStateContext, current_time: Time) -> Odometry:
    while not rospy.is_shutdown():
        dt = (current_time - context.prev_time).to_sec()

        # Вычисление линейной и угловой скорости
        linear_velocity = wheel_radius * (context.v_right + context.v_left) / 2.0
        angular_velocity = wheel_radius * (context.v_right - context.v_left) / wheel_separation_length

        # Обновление позиции
        delta_x = linear_velocity * math.cos(context.th) * dt
        delta_y = linear_velocity * math.sin(context.th) * dt
        delta_th = angular_velocity * dt

        context.x += delta_x
        context.y += delta_y
        context.th += delta_th

        # Создание сообщения Odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Позиция
        odom.pose.pose.position.x = context.x
        odom.pose.pose.position.y = context.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, context.th)

        # Скорость
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Ковариации (настройте в зависимости от точности)
        odom.pose.covariance = [1e-3, 0, 0, 0, 0, 0,
                                0, 1e-3, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 1e-2]
        odom.twist.covariance = [1e-3, 0, 0, 0, 0, 0,
                                 0, 1e-3, 0, 0, 0, 0,
                                 0, 0, 1e6, 0, 0, 0,
                                 0, 0, 0, 1e6, 0, 0,
                                 0, 0, 0, 0, 1e6, 0,
                                 0, 0, 0, 0, 0, 1e-2]

        # Публикация одометрии
        return odom


def joint_state_publisher(all_joints: AllJointsState):
    pub_joint = rospy.Publisher('/joint_states', JointState, queue_size=10)
    pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10 Hz
    context = AllStateContext()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        wheel_joint_state = create_wheel_joint_state(all_joints, context, current_time)
        odom = odometry_state(all_joints, context, current_time)
        context.prev_time = current_time

        # Публикация TF
        br.sendTransform((context.x, context.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, context.th),
                         current_time,
                         "base_link",
                         "odom")
        
        pub_joint.publish(wheel_joint_state)
        pub_odom.publish(odom)
        rate.sleep()


def velocity_subscriber(all_joints: AllJointsState):
    def right_velocity_callback(msg):
        all_joints.current_right_rotation_speed = msg.data

    def left_velocity_callback(msg):
        all_joints.current_left_rotation_speed = -1.0 * msg.data

    rospy.Subscriber("/pbot/right_wheel/current_velocity", Float64, right_velocity_callback)
    rospy.Subscriber("/pbot/left_wheel/current_velocity", Float64, left_velocity_callback)


def start():
    rospy.init_node('pbot_jointer')
    joints = AllJointsState()
    # Запускаем подписчики в отдельном потоке
    threading.Thread(target=velocity_subscriber, args=(joints,)).start()
    # Запускаем публикатор
    rospy.loginfo("Jointer node started")

    try:
        joint_state_publisher(joints)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    start()

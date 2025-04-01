#!/usr/bin/env python
import math

import rospy
from rospy import Time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading

ANGLE_LIMIT = 2 * math.pi


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


def joint_state_publisher(all_joints: AllJointsState, context: AllStateContext):
    pub_joint = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        wheel_joint_state = create_wheel_joint_state(all_joints, context, current_time)
        context.prev_time = current_time
        pub_joint.publish(wheel_joint_state)
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

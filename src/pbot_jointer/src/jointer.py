#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import threading

rad2pwm_k = 0.044
max_radian_speed = 100 * rad2pwm_k
min_radian_speed = -100 * rad2pwm_k


class AllJointsState:
    def __init__(self):
        # self.left_motor = motor.left_motor()
        # self.right_motor = motor.right_motor()
        self.current_right_rotation_speed = 0.
        self.current_left_rotation_speed = 0.
        # self._current_right_command_value = 0
        # self._current_left_command_value = 0
        # self._current_cmd_x_speed = 0.
        # self._current_cmd_z_angular_speed = 0.

        # rospy.init_node('pbot_jointer')
        # rospy.Subscriber("/pbot/right_wheel/current_velocity", Float64, self.callback_right_wheel)
        # rospy.Subscriber("/pbot/left_wheel/current_velocity", Float64, self.callback_left_wheel)
        # rospy.Subscriber("/pbot/cmd_vel", Twist, self.callback_cmd_vel)

        # self.pub_right_wheel = rospy.Publisher("/pbot/right_wheel/current_velocity", Float64, queue_size=10)
        # self.pub_left_wheel = rospy.Publisher("/pbot/left_wheel/current_velocity", Float64, queue_size=10)
        # self.pub_target_right_wheel = rospy.Publisher("/pbot/right_wheel/target_velocity", Float64, queue_size=10)
        # self.pub_target_left_wheel = rospy.Publisher("/pbot/left_wheel/target_velocity", Float64, queue_size=10)

    def callback_right_wheel(self, msg: Float64):
        self.current_right_rotation_speed = msg.data

    def callback_left_wheel(self, msg: Float64):
        self.current_left_rotation_speed = -1.0 * msg.data


def joint_state_publisher(all_joints: AllJointsState):
    pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    # Инициализация сообщения JointState
    joint_state = JointState()
    joint_state.name = ['pbot_lf_w', 'pbot_lr_w', 'pbot_rf_w', 'pbot_rr_w']
    joint_state.position = [0.0, 0.0, 0.0, 0.0]
    joint_state.velocity = [0.0, 0.0, 0.0, 0.0]
    joint_state.effort = [0.0, 0.0, 0.0, 0.0]

    # Предыдущее время для расчёта положения
    prev_time = rospy.Time.now()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        delta_t = (current_time - prev_time).to_sec()

        # Обновляем положение колёс
        right_position_delta = all_joints.current_right_rotation_speed * delta_t
        left_position_delta = all_joints.current_left_rotation_speed * delta_t
        joint_state.position[0] += left_position_delta
        joint_state.position[1] += left_position_delta
        joint_state.position[2] += right_position_delta
        joint_state.position[3] += right_position_delta

        # Обновляем скорость
        joint_state.velocity = [all_joints.current_left_rotation_speed, all_joints.current_left_rotation_speed,
                                all_joints.current_right_rotation_speed, all_joints.current_right_rotation_speed]

        # Обновляем время
        prev_time = current_time

        joint_state.header.stamp = current_time

        # Публикуем сообщение
        pub.publish(joint_state)

        rate.sleep()

def velocity_subscriber(all_joints: AllJointsState):
    def right_velocity_callback(msg):
        all_joints.current_right_rotation_speed = msg.data

    def left_velocity_callback(msg):
        all_joints.current_left_rotation_speed = msg.data

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

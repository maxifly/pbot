#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from std_msgs.msg import Int16
from std_msgs.msg import Int8
from pbot_joy_converter.cfg import joy_converterConfig
from pbot_beeper.msg import Beep


class JoyConverter:

    def __init__(self):
        rospy.init_node('joy_converter')

        # Настройка сервера динамической реконфигурации
        self.srv = Server(joy_converterConfig, self.config_callback)

        # Настройка подписчика на сообщения Joy
        self.joy_sub = rospy.Subscriber('/pbot/joy', Joy, self.joy_callback)

        # Настройка издателя для сообщений Twist
        self.twist_pub = rospy.Publisher('/pbot/cmd_vel', Twist, queue_size=10)

        self.cam_h_pub = rospy.Publisher('/pbot/cam_h_servo/change_pos', Int16, queue_size=10)
        self.cam_v_pub = rospy.Publisher('/pbot/cam_v_servo/change_pos', Int16, queue_size=10)
        self.cam_v_reset_pos = rospy.Publisher('/pbot/cam_v_servo/reset_pos', Int16, queue_size=10)

        self.beeper_pub = rospy.Publisher('/pbot/beeper_topic', Beep, queue_size=10)
        self.special_pub = rospy.Publisher('/pbot/special_mode', Int8, queue_size=10)

        # Начальные значения ограничений
        self.max_linear_velocity = 0.5  # м/с
        self.max_angular_velocity = 1.0  # рад/с

    def config_callback(self, config, level):
        """
        Обработчик обратного вызова для сервера динамической реконфигурации.

        Args:
            config (joy_to_twist_converter.cfg.JoyToTwistConfig): Новые значения конфигурации.
            level (int): Уровень изменения конфигурации.
        """

        self.max_linear_velocity = config.max_linear_velocity
        self.max_angular_velocity = config.max_angular_velocity
        rospy.loginfo("Reconfigure request: max_linear_velocity=%f, max_angular_velocity=%f" % (
            config.max_linear_velocity, config.max_angular_velocity))
        return config

    def joy_callback(self, data):
        """
        Обработчик обратного вызова, получающий сообщения Joy и публикующий сообщения Twist.

        Args:
            data (sensor_msgs.msg.Joy): Сообщение Joy, полученное от подписчика.
        """
        if data.axes[2] == 0. and data.axes[3] == 0.:
            # Создаем объект Twist для хранения данных о скорости и угловой скорости
            twist = Twist()

            # Маппинг кнопок и осей джойстика на данные Twist
            twist.linear.x = data.axes[1] * self.max_linear_velocity
            # twist.linear.y = data.axes[0] * 0.5  # Скорость движения влево/вправо
            twist.angular.z = -1. * data.axes[0] * self.max_angular_velocity

            # Применение ограничений на максимальные скорости
            twist.linear.x = max(min(twist.linear.x, self.max_linear_velocity), -self.max_linear_velocity)
            twist.angular.z = max(min(twist.angular.z, self.max_angular_velocity), -self.max_angular_velocity)

            # Публикуем сообщение Twist
            self.twist_pub.publish(twist)

        if data.axes[2] != 0. or data.axes[3] != 0.:
            twist1 = Twist()
            twist1.linear.x = 0.
            twist1.angular.z = 0.15
            self.twist_pub.publish(twist)

        if data.buttons[0] == 1:
            self.cam_v_pub.publish(1)
        if data.buttons[4] == 1:
            self.cam_v_pub.publish(-1)

        if data.buttons[3] == 1:
            self.cam_h_pub.publish(1)
        if data.buttons[1] == 1:
            self.cam_h_pub.publish(-1)
        if data.buttons[11] == 1:
            self.cam_v_reset_pos.publish(1)
        if data.buttons[10] == 1:
            beeper = Beep()
            beeper.type = 1
            self.beeper_pub.publish(beeper)
        if data.buttons[6] == 1:
            self.special_pub.publish(1)
        if data.buttons[7] == 1:
            self.special_pub.publish(2)


if __name__ == '__main__':
    converter = JoyConverter()
    rospy.spin()

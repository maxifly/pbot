#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from pbot_joy_converter.cfg import joy_converterConfig


class JoyConverter:

    def __init__(self):
        rospy.init_node('joy_converter')

        # Настройка сервера динамической реконфигурации
        self.srv = Server(joy_converterConfig, self.config_callback)

        # Настройка подписчика на сообщения Joy
        self.joy_sub = rospy.Subscriber('/pbot/joy', Joy, self.joy_callback)

        # Настройка издателя для сообщений Twist
        self.twist_pub = rospy.Publisher('/pbot/cmd_vel', Twist, queue_size=10)

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


if __name__ == '__main__':
    converter = JoyConverter()
    rospy.spin()

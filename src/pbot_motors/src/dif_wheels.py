import math

# Константы
WHEEL_RADIUS = 0.0335  # Диаметр колеса в метрах
WHEEL_DISTANCE = 0.135  # Расстояние между колесами в метрах

def calculate_wheel_speeds(linear_velocity, angular_velocity):
  """
  Вычисляет угловые скорости вращения колес.

  Args:
    linear_velocity: Линейная скорость тележки в м/с.
    angular_velocity: Угловая скорость тележки в радианах/с.

  Returns:
    Кортеж из двух значений: угловая скорость левого колеса, угловая скорость правого колеса (в радианах/с).
  """

  
  # Вычисление скорости движения центра тележки
  center_velocity = linear_velocity

  # Если линейная скорость равна нулю, ускоряем поворот вращая колеса в разные стороны
  if abs(linear_velocity) < 0.01:
    target_left_wheel_angular_speed = (angular_velocity * WHEEL_DISTANCE / 2) / WHEEL_RADIUS
    target_right_wheel_angular_speed = -(angular_velocity * WHEEL_DISTANCE / 2) / WHEEL_RADIUS
  else:
    # Вычисление угловых скоростей колес
    target_left_wheel_angular_speed = (center_velocity + (angular_velocity * WHEEL_DISTANCE / 2)) / WHEEL_RADIUS
    target_right_wheel_angular_speed = (center_velocity - (angular_velocity * WHEEL_DISTANCE / 2)) / WHEEL_RADIUS

  return target_left_wheel_angular_speed, target_right_wheel_angular_speed
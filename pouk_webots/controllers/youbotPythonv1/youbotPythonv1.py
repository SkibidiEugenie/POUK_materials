from controller import Supervisor
import math
import sys

TIME_STEP = 32
WHEEL_RADIUS = 0.05

robot = Supervisor()
robot_node = robot.getSelf()
if robot_node is None:
    print("ОШИБКА: Включите supervisor TRUE!")
    sys.exit(1)

translation_field = robot_node.getField("translation")
rotation_field = robot_node.getField("rotation")

wheels = []

def step():
    if robot.step(TIME_STEP) == -1:
        sys.exit(0)

def init_wheels():
    for i in range(4):
        w = robot.getDevice(f"wheel{i + 1}")
        if w is None:
            print(f"ОШИБКА: Не найдено колесо wheel{i + 1}")
            return False
        w.setPosition(float('inf'))
        w.setVelocity(0.0)
        wheels.append(w)
    return True

def set_wheel_speeds(v1, v2, v3, v4):
    for w, v in zip(wheels, (v1, v2, v3, v4)):
        w.setVelocity(v)

def normalize(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def move_forward(distance_m, speed):
    wheel_speed = speed / WHEEL_RADIUS
    duration = distance_m / speed
    start_time = robot.getTime()

    set_wheel_speeds(wheel_speed, wheel_speed, wheel_speed, wheel_speed)
    while robot.getTime() - start_time < duration:
        step()

    set_wheel_speeds(0.0, 0.0, 0.0, 0.0)
    step()
    print(f"Проехали {distance_m:.2f} м")
    return True

def turn_in_place(target_angle_deg, speed):
    wheel_speed = speed / WHEEL_RADIUS

    start_angle = normalize(rotation_field.getSFRotation()[3])
    target_angle = normalize(start_angle + math.radians(target_angle_deg))

    print(f"Начинаем поворот. Текущий угол: {math.degrees(start_angle):.1f} град, "
          f"целевой: {math.degrees(target_angle):.1f} град")

    if target_angle_deg > 0:
        set_wheel_speeds(wheel_speed, -wheel_speed, wheel_speed, -wheel_speed)
    else:
        set_wheel_speeds(-wheel_speed, wheel_speed, -wheel_speed, wheel_speed)

    log_counter = 0
    while True:
        step()
        current_angle = normalize(rotation_field.getSFRotation()[3])
        diff = normalize(target_angle - current_angle)

        if log_counter % 10 == 0:
            print(f"Угол: {math.degrees(current_angle):.1f} град, "
                  f"осталось: {math.degrees(diff):.1f} град")
        log_counter += 1

        if abs(diff) < 0.02:
            break

    set_wheel_speeds(0.0, 0.0, 0.0, 0.0)
    step()
    print("Поворот завершен!")

def visit_printer(numb_of_table, id_):
    move_forward(2.0, 0.5)
    turn_in_place(90.0, 0.5)
    # Подход к столу
    move_forward(2.3 + 1.0 * (numb_of_table - 1), 0.5)
    turn_in_place(-90.0, 0.5)
    # Подход к принтеру
    move_forward(0.35 + 0.48 * (id_ - 1), 0.5)
    return True

def main():
    if not init_wheels():
        return
    step()

    start = translation_field.getSFVec3f()
    print(f"СТАРТ: X={start[0]:.3f}, Y={start[1]:.3f}, Z={start[2]:.3f}")

    visit_printer(1, 2)

    while True:
        step()


main()
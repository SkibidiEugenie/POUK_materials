#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 32

static WbDeviceTag wheels[4];
static WbNodeRef robot_node;
static WbFieldRef translation_field;
static WbFieldRef rotation_field;

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

// Инициализация колес
static void init_wheels() {
  wheels[0] = wb_robot_get_device("wheel1");
  wheels[1] = wb_robot_get_device("wheel2");
  wheels[2] = wb_robot_get_device("wheel3");
  wheels[3] = wb_robot_get_device("wheel4");
  
  for (int i = 0; i < 4; i++) {
    if (wheels[i] == 0) {
      printf("ОШИБКА: Не найдено колесо wheel%d\n", i+1);
    } else {
      wb_motor_set_position(wheels[i], INFINITY);
      wb_motor_set_velocity(wheels[i], 0.0);
    }
  }
}

// Установка скорости всех колес
static void set_wheel_speeds(double v1, double v2, double v3, double v4) {
  wb_motor_set_velocity(wheels[0], v1);
  wb_motor_set_velocity(wheels[1], v2);
  wb_motor_set_velocity(wheels[2], v3);
  wb_motor_set_velocity(wheels[3], v4);
}

// Движение вперед на заданное расстояние (в метрах)
static void move_forward(double distance_m, double speed) {
  double wheel_speed = speed / 0.05; 
  double duration = distance_m / speed;
  double start_time = wb_robot_get_time();
  
  set_wheel_speeds(wheel_speed, wheel_speed, wheel_speed, wheel_speed);
  
  while (wb_robot_get_time() - start_time < duration) {
    step();
  }
  
  set_wheel_speeds(0.0, 0.0, 0.0, 0.0);
  step();
  printf("Проехали %.2f м\n", distance_m);
}

// Поворот на месте с контролем угла через Supervisor
// target_angle_deg > 0  -> поворот налево (против часовой)
// target_angle_deg < 0  -> поворот направо (по часовой)
static void turn_in_place(double target_angle_deg, double speed) {
  double wheel_speed = speed / 0.05; 
  
  const double *rot = wb_supervisor_field_get_sf_rotation(rotation_field);
  double start_angle = rot[3];
  
  // Приводим начальный угол к диапазону [-PI, PI]
  while (start_angle > M_PI) start_angle -= 2.0 * M_PI;
  while (start_angle < -M_PI) start_angle += 2.0 * M_PI;
  
  double target_angle = start_angle + (target_angle_deg * M_PI / 180.0);
  // Нормализуем целевой угол
  while (target_angle > M_PI) target_angle -= 2.0 * M_PI;
  while (target_angle < -M_PI) target_angle += 2.0 * M_PI;
  
  printf("Начинаем поворот. Текущий угол: %.1f град, целевой: %.1f град\n", 
         start_angle * 180.0 / M_PI, target_angle * 180.0 / M_PI);
  
  // ВНИМАНИЕ: Здесь мы меняем скорости местами, чтобы робот крутился в нужную сторону
  if (target_angle_deg > 0) {
    // Поворот налево (против часовой)
    set_wheel_speeds(wheel_speed, -wheel_speed, wheel_speed, -wheel_speed);
  } else {
    // Поворот направо (по часовой)
    set_wheel_speeds(-wheel_speed, wheel_speed, -wheel_speed, wheel_speed);
  }
  
  int log_counter = 0;
  while (true) {
    step();
    
    const double *current_rot = wb_supervisor_field_get_sf_rotation(rotation_field);
    double current_angle = current_rot[3];
    
    // Нормализуем текущий угол
    while (current_angle > M_PI) current_angle -= 2.0 * M_PI;
    while (current_angle < -M_PI) current_angle += 2.0 * M_PI;
    
    double diff = target_angle - current_angle;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    
    if (log_counter % 10 == 0) {
      printf("Угол: %.1f град, осталось: %.1f град\n", current_angle * 180.0 / M_PI, diff * 180.0 / M_PI);
    }
    log_counter++;
    
    if (fabs(diff) < 0.02) { // Уменьшили допуск для точности
      break;
    }
  }
  
  set_wheel_speeds(0.0, 0.0, 0.0, 0.0);
  step();
  printf("Поворот завершен!\n");
}

int main(int argc, char **argv) {
  wb_robot_init();

  robot_node = wb_supervisor_node_get_self();
  if (robot_node == NULL) {
    printf("ОШИБКА: Включите supervisor TRUE!\n");
    return 1;
  }
  
  translation_field = wb_supervisor_node_get_field(robot_node, "translation");
  rotation_field = wb_supervisor_node_get_field(robot_node, "rotation");
  
  init_wheels();
  step();

  const double *start = wb_supervisor_field_get_sf_vec3f(translation_field);
  printf("СТАРТ: X=%.3f, Y=%.3f, Z=%.3f\n", start[0], start[1], start[2]);

  // УМЕНЬШИЛИ ДИСТАНЦИЮ ПЕРЕД ПОВОРОТОМ (было 2.6, стало 2.2)
  printf("\n--- Едем к точке 1 ---\n");
  move_forward(2.2, 0.5); // <-- УМЕНЬШИЛИ РАССТОЯНИЕ

  // ПОВОРОТ НАЛЕВО (положительный угол)
  printf("\n--- Поворот на 90 град налево ---\n");
  turn_in_place(90.0, 0.5); // <-- ПОЛОЖИТЕЛЬНЫЙ УГОЛ = НАЛЕВО

  printf("\n--- Едем к точке 2 ---\n");
  move_forward(2.6, 0.5);

  printf("\n=== ГОТОВО! ===\n");

  while (true) step();
  return 0;
}
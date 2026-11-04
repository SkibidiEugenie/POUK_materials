/*
 * Контроллер для youBot с использованием Supervisor.
 * Плавно перемещает робота, сохраняя высоту, и логирует координаты.
 */

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 32

static WbNodeRef robot_node;
static WbFieldRef translation_field;
static WbFieldRef rotation_field;

// Вспомогательная функция для шага симуляции
static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

// Функция для плавного перемещения в заданную точку по плоскости XZ
// Высота Y сохраняется автоматически с первого шага
static void move_to_position(double target_x, double target_z, double speed) {
  const double *start_pos = wb_supervisor_field_get_sf_vec3f(translation_field);
  double fixed_y = start_pos[1]; // Запоминаем текущую высоту
  
  printf("Начинаем движение к X=%.3f, Z=%.3f (высота Y=%.3f)\n", target_x, target_z, fixed_y);
  
  int log_counter = 0;
  
  while (true) {
    const double *current_pos = wb_supervisor_field_get_sf_vec3f(translation_field);
    
    double dx = target_x - current_pos[0];
    double dz = target_z - current_pos[2];
    double distance = sqrt(dx*dx + dz*dz);
    
    // Логируем каждые 10 шагов (примерно каждые 0.32 секунды)
    if (log_counter % 10 == 0) {
      printf("Текущая позиция: X=%.4f, Y=%.4f, Z=%.4f | До цели: %.3f м\n", 
             current_pos[0], current_pos[1], current_pos[2], distance);
    }
    log_counter++;
    
    if (distance < 0.005) {
      printf("Достигли точки! Остановка.\n");
      break;
    }
    
    double step_distance = speed * (TIME_STEP / 1000.0);
    if (step_distance > distance) step_distance = distance;
    
    double new_x = current_pos[0] + (dx / distance) * step_distance;
    double new_z = current_pos[2] + (dz / distance) * step_distance;
    
    // Принудительно ставим фиксированную высоту
    wb_supervisor_field_set_sf_vec3f(translation_field, (double[]){new_x, fixed_y, new_z});
    
    step();
  }
}

// Функция для плавного поворота робота вокруг оси Y
static void turn_robot(double target_angle_deg) {
  double target_angle_rad = target_angle_deg * M_PI / 180.0;
  
  printf("Начинаем поворот на %.1f градусов\n", target_angle_deg);
  
  int log_counter = 0;
  
  while (true) {
    const double *rot = wb_supervisor_field_get_sf_rotation(rotation_field);
    double current_angle_rad = rot[3];
    
    // Приводим углы к диапазону [-PI, PI]
    double diff = target_angle_rad - current_angle_rad;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    
    if (log_counter % 10 == 0) {
      printf("Текущий угол: %.1f град, осталось: %.1f град\n", 
             current_angle_rad * 180.0 / M_PI, diff * 180.0 / M_PI);
    }
    log_counter++;
    
    if (fabs(diff) < 0.0087) {
      printf("Поворот завершен.\n");
      break;
    }
    
    double step_angle = 0.02;
    if (fabs(diff) < step_angle) step_angle = fabs(diff);
    
    if (diff > 0) current_angle_rad += step_angle;
    else current_angle_rad -= step_angle;
    
    // Сохраняем текущую ось вращения, меняем только угол
    // (В Webots робот по умолчанию вращается вокруг Y)
    wb_supervisor_field_set_sf_rotation(rotation_field, (double[]){0.0, 1.0, 0.0, current_angle_rad});
    
    step();
  }
}

int main(int argc, char **argv) {
  wb_robot_init();

  robot_node = wb_supervisor_node_get_self();
  if (robot_node == NULL) {
    printf("ОШИБКА: Робот не является супервизором! Включите supervisor TRUE в свойствах.\n");
    return 1;
  }
  
  translation_field = wb_supervisor_node_get_field(robot_node, "translation");
  rotation_field = wb_supervisor_node_get_field(robot_node, "rotation");

  printf("=== ЗАПУСК КОНТРОЛЛЕРА ===\n");
  step(); // Ждем один шаг для инициализации

  // Выводим стартовые координаты
  const double *start_pos = wb_supervisor_field_get_sf_vec3f(translation_field);
  const double *start_rot = wb_supervisor_field_get_sf_rotation(rotation_field);
  printf("СТАРТОВЫЕ КООРДИНАТЫ: X=%.4f, Y=%.4f, Z=%.4f\n", start_pos[0], start_pos[1], start_pos[2]);
  printf("СТАРТОВЫЙ УГОЛ: %.2f рад (ось: %.2f, %.2f, %.2f)\n", start_rot[3], start_rot[0], start_rot[1], start_rot[2]);

  // 1. Движение к первой точке (X, Z)
  printf("\n--- ЭТАП 1: Движение к точке 1 ---\n");
  move_to_position(-0.299394, -1.93987, 1.0); // Скорость 1 м/с

  // 2. Поворот на 90 градусов против часовой стрелки
  printf("\n--- ЭТАП 2: Поворот на 90 градусов ---\n");
  const double *current_rot = wb_supervisor_field_get_sf_rotation(rotation_field);
  double current_angle = current_rot[3];
  
  // Поворот на +90 градусов (против часовой) = +PI/2
  double target_angle = current_angle + (M_PI / 2.0);
  if (target_angle > M_PI) target_angle -= 2.0 * M_PI;
  
  turn_robot(target_angle * 180.0 / M_PI);

  // 3. Движение ко второй точке (X, Z)
  printf("\n--- ЭТАП 3: Движение к точке 2 ---\n");
  move_to_position(-0.299742, 0.67013, 1.0);

  printf("\n=== ЗАДАЧА ВЫПОЛНЕНА! ===\n");

  while (true) {
    step();
  }

  wb_robot_cleanup();
  return 0;
}
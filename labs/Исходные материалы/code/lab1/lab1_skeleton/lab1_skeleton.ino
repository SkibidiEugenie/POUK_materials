// ============================================================================
// ЛР1. Каркас прошивки: встроенное управление 5-звенным манипулятором
// Плата: OpenCM9.04 (STM32F103), приводы: 6 x Dynamixel AX-12A, протокол 1.0
//
// Слои (сверху вниз по файлу):
//   [COMM]  команды с ПК и телеметрия
//   [SCEN]  сценарии: точка, окружность, контроль достижения
//   [KIN]   кинематика: solveIK  (<-- ВАШ ОСНОВНОЙ TODO)
//   [UNIT]  преобразование единиц: радианы <-> отсчёты 0..1023
//   [DRV]   драйвер шины Dynamixel
//
// Команды в мониторе порта (57600, окончание строки '\n'):
//   S              - скан приводов 1..6 (ping, позиция)
//   P <id> <pos>   - одиночный привод в позицию 0..1023
//   G <x> <y> <z>  - схват в точку (мм), через ОЗК
//   C              - окружность из circle_traj.h с контролем достижения
//   H              - домой (все в 512)
// ============================================================================
#include <Dynamixel2Arduino.h>
#include "circle_traj.h"     // массив уставок окружности (генерирует lab1_traj_and_plots.py)

// ---------------- [DRV] драйвер шины -----------------------------------------
#define DXL_SERIAL   Serial3          // порт шины DXL на плате стенда
const uint8_t DXL_DIR_PIN = 22;
const float   DXL_PROTOCOL = 1.0;
const uint8_t N_JOINTS = 6;           // ID 1..5 - звенья, ID 6 - схват

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void busInit() {
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  for (uint8_t id = 1; id <= N_JOINTS; id++) dxl.setOperatingMode(id, OP_POSITION);
}

bool busPing(uint8_t id)              { return dxl.ping(id); }
int32_t busReadPos(uint8_t id)        { return dxl.getPresentPosition(id); }
void busWritePos(uint8_t id, int32_t p){ dxl.setGoalPosition(id, p); }
void busWriteVel(uint8_t id, int32_t v){ dxl.setGoalVelocity(id, v); }

// TODO(*1): sync write - один широковещательный кадр вместо шести одиночных.
// Замерьте micros() вокруг рассылки до и после. См. syncWrite API Dynamixel2Arduino.

// ---------------- [UNIT] единицы ---------------------------------------------
const float DEL = 0.00511327f;        // рад в одном отсчёте (~300°/1024)

int32_t rad2ticks(float a)  { return 512 + (int32_t)lroundf(a / DEL); }
float   ticks2rad(int32_t t){ return (t - 512) * DEL; }

// Пределы суставов в ОТСЧЁТАХ. Стартовые значения покрывают штатную
// окружность из circle_traj.h; уточните экспериментально на шаге 2
// (одиночными движениями!) и обоснуйте выбранные значения в отчёте.
int32_t JOINT_MIN[N_JOINTS] = {162, 262, 262, 362, 162, 312};
int32_t JOINT_MAX[N_JOINTS] = {862, 762, 892, 892, 862, 712};

bool inLimits(uint8_t idx, int32_t t) { return t >= JOINT_MIN[idx] && t <= JOINT_MAX[idx]; }

// ---------------- [KIN] кинематика -------------------------------------------
// Длины условных звеньев стенда, м (проверьте по своей сборке!)
const float L1 = 0.103f, L2 = 0.1045f, L3 = 0.079f, L4 = 0.169f;

// ОЗК при вертикальном схвате (приложение А к ЛР1, §А.4.2). Вход: x,y,z в МЕТРАХ.
// Выход: пять углов (рад) в out[0..4]; return false, если цель недостижима
// или нарушены пределы суставов.
bool solveIK(float x, float y, float z, float out[5]) {
  // ---- перенос системы координат
  float k   = sqrtf(x * x + y * y);
  float z_r = z + (L4 - L1);
  float d   = sqrtf(k * k + z_r * z_r);

  // TODO шаг 3.1: проверка достижимости ДО acos:
  //   аргументы acos ниже обязаны лежать в [-1; 1];
  //   эквивалентно |L2-L3| <= d <= L2+L3 (плюс запас на пределы суставов).
  //   При нарушении: Serial.println("IK: unreachable"); return false;

  // TODO шаг 3.2: сами формулы (приложение А, §А.4.2; atan2, не atan!):
  //   out[0] = atan2(y, x);                                  // основание
  //   out[1] = PI/2 - (atan2(z_r, k) + acosf((d*d + L2*L2 - L3*L3) / (2*d*L2)));
  //   out[2] = PI   -  acosf((-d*d + L2*L2 + L3*L3) / (2*L2*L3));
  //   out[3] = PI   - (out[1] + out[2]);                     // вертикальность схвата
  //   out[4] = out[0];                                       // компенсация поворота

  // TODO шаг 3.3: перевести в отсчёты (rad2ticks) и проверить inLimits()
  //   для каждого сустава; при нарушении - сообщение и return false.

  (void)d; (void)out;                 // удалить после реализации
  Serial.println("IK: not implemented");
  return false;
}

// ---------------- [SCEN] сценарии --------------------------------------------
const int32_t EPS_TICKS   = 16;       // допуск достижения (обоснуйте свой!)
const uint32_t REACH_TO_MS = 4000;    // тайм-аут достижения точки

void applyAngles(const float a[5], int32_t vel) {
  for (uint8_t i = 0; i < 5; i++) {
    busWriteVel(i + 1, vel);
    busWritePos(i + 1, rad2ticks(a[i]));
  }
}

// Ожидание достижения: true - дошли, false - тайм-аут.
bool waitReach(const int32_t target[5]) {
  uint32_t t0 = millis();
  while (millis() - t0 < REACH_TO_MS) {
    // TODO шаг 4: прочитать busReadPos(1..5), найти max|факт-цель|;
    //   если <= EPS_TICKS - return true;
    //   печатать телеметрию не чаще раза в 50 мс (millis-планировщик, БЕЗ delay).
  }
  Serial.println("REACH: timeout");
  return false;
}

void cmdGoto(float x_mm, float y_mm, float z_mm) {
  float a[5];
  if (!solveIK(x_mm * 0.001f, y_mm * 0.001f, z_mm * 0.001f, a)) return;
  applyAngles(a, 100);
  int32_t tgt[5];
  for (uint8_t i = 0; i < 5; i++) tgt[i] = rad2ticks(a[i]);
  waitReach(tgt);
}

void cmdCircle() {
  Serial.println("CIRCLE: start");
  for (uint16_t p = 0; p < TRAJ_N; p++) {
    int32_t tgt[5];
    for (uint8_t j = 0; j < 5; j++) {
      tgt[j] = CIRCLE_TRAJ[j][p];
      busWriteVel(j + 1, 80);
      busWritePos(j + 1, tgt[j]);        // TODO(*1): заменить на sync write
    }
    if (!waitReach(tgt)) Serial.print("CIRCLE: skip point "), Serial.println(p);
    telemetry();
  }
  Serial.println("CIRCLE: done");
}

void cmdHome() {
  for (uint8_t id = 1; id <= 5; id++) { busWriteVel(id, 60); busWritePos(id, 512); }
}

// ---------------- [COMM] связь с ПК ------------------------------------------
void telemetry() {
  Serial.print("T,"); Serial.print(millis());
  for (uint8_t id = 1; id <= N_JOINTS; id++) { Serial.print(','); Serial.print(busReadPos(id)); }
  Serial.println();
}
// TODO(*3): телеметрия строгим периодом 50 мс по millis-планировщику; оцените джиттер.

void cmdScan() {
  for (uint8_t id = 1; id <= N_JOINTS; id++) {
    Serial.print("ID "); Serial.print(id);
    if (busPing(id)) { Serial.print(": ok, pos="); Serial.println(busReadPos(id)); }
    else             { Serial.println(": NO RESPONSE"); }
  }
}

void parseLine(String s) {
  s.trim();
  if (s.length() == 0) return;
  switch (s[0]) {
    case 'S': cmdScan(); break;
    case 'H': cmdHome(); break;
    case 'C': cmdCircle(); break;
    case 'P': {
      int id, pos;
      if (sscanf(s.c_str(), "P %d %d", &id, &pos) == 2 &&
          id >= 1 && id <= N_JOINTS && inLimits(id - 1, pos)) {
        busWriteVel(id, 80); busWritePos(id, pos);
      } else Serial.println("P: bad args or out of limits");
      break; }
    case 'G': {
      float x, y, z;
      if (sscanf(s.c_str(), "G %f %f %f", &x, &y, &z) == 3) cmdGoto(x, y, z);
      else Serial.println("G: bad args");
      break; }
    default: Serial.println("cmd? S|P id pos|G x y z|C|H");
  }
}

void setup() {
  Serial.begin(57600);
  Serial.setTimeout(50);
  busInit();
  Serial.println("LAB1 skeleton ready. Commands: S|P|G|C|H");
  cmdScan();                          // стартовая самопроверка
}

void loop() {
  if (Serial.available()) parseLine(Serial.readStringUntil('\n'));
  // Ничего блокирующего здесь быть не должно. delay() запрещён (приложение А, §А.9; пособие, §1.7.2).
}

// ============================================================================
// ЛР2. Прошивка стенда: протокол v2 поверх UART к одноплатнику.
// Плата: OpenCM9.04 на STEM Board 2; приводы: 6 x Dynamixel AX-12A (протокол 1.0).
//
// Файлы скетча:
//   pouk_proto.h/.cpp  — CRC-32, COBS, автомат приёмника      (переносимый C++98)
//   mcu_logic.h/.cpp   — режимы, сторожевой таймер, команды   (переносимый C++98, TODO шаги 6–7)
//   pouk_mcu.ino       — этот файл: шина Dynamixel и порт       (привязка к Arduino)
// Логика проверяется без платы: host_test/ (инструкция ЛР2, шаги 6–7).
// ============================================================================
#include <Dynamixel2Arduino.h>
#include "pouk_proto.h"
#include "mcu_logic.h"

// ---------------- конфигурация ----------------------------------------------
#define LINK_SERIAL   Serial1        // STEM Board 2: Serial1 <-> NanoPi /dev/ttyS1
#define LINK_BAUD     115200
#define DEBUG_SERIAL  Serial         // USB; закомментируйте, если канал идёт по USB
// Вариант без одноплатника (мост на ПК по USB): LINK_SERIAL = Serial, DEBUG_SERIAL убрать.

#define DXL_SERIAL    Serial3        // шина приводов на плате стенда
const uint8_t DXL_DIR_PIN = 22;
const float   DEL = 0.00511327f;     // рад в одном отсчёте AX-12A
const float   SPEED_UNIT_MRAD_S = 11.62f;   // единица Moving Speed AX-12A ≈ 0,111 об/мин

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ---------------- [DRV] шина приводов ---------------------------------------
class DxlBus : public pouk::BusIface {
 public:
  bool setTorque(bool on) {
    bool ok = true;
    for (uint8_t id = 1; id <= pouk::N_JOINTS; id++) ok = (on ? dxl.torqueOn(id) : dxl.torqueOff(id)) && ok;
    return ok;
  }
  bool writeGoals(const int16_t q[pouk::N_JOINTS], uint16_t vmax) {
    int32_t spd = (int32_t)(vmax / SPEED_UNIT_MRAD_S + 0.5f);
    if (spd < 1) spd = 1;                       // 0 у AX-12A означает «без ограничения скорости»
    if (spd > 1023) spd = 1023;
    bool ok = true;
    for (uint8_t j = 0; j < pouk::N_JOINTS; j++) {
      int32_t t = 512 + (int32_t)lroundf(q[j] / 1000.0f / DEL);
      if (t < 0) t = 0;
      if (t > 1023) t = 1023;
      ok = dxl.setGoalVelocity(j + 1, spd) && ok;
      ok = dxl.setGoalPosition(j + 1, t) && ok;
    }
    return ok;
  }
  bool readPositions(int16_t q[pouk::N_JOINTS]) {
    for (uint8_t j = 0; j < pouk::N_JOINTS; j++) {
      int32_t t = (int32_t)dxl.getPresentPosition(j + 1);
      if (dxl.getLastLibErrCode() != DXL_LIB_OK) return false;
      q[j] = (int16_t)lroundf((t - 512) * DEL * 1000.0f);
    }
    return true;
  }
};

// ---------------- [COMM] канал к одноплатнику -------------------------------
class SerialTx : public pouk::LinkTx {
 public:
  void write(const uint8_t *data, size_t len) { LINK_SERIAL.write(data, len); }
};

DxlBus bus;
SerialTx tx;
pouk::McuConfig cfg;
pouk::McuLogic *logic = 0;
uint8_t last_state = 255;

void setup() {
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.begin(115200);
#endif
  LINK_SERIAL.begin(LINK_BAUD);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(1.0);
  for (uint8_t id = 1; id <= pouk::N_JOINTS; id++) dxl.setOperatingMode(id, OP_POSITION);

  pouk::defaultConfig(cfg);
  // cfg.wd_timeout_ms = ...;   // ЛР2, шаг 7: расчётное значение T_wd
  uint8_t session = (uint8_t)(micros() ^ (analogRead(0) << 3)) | 1;
  static pouk::McuLogic instance(bus, tx, cfg, session);
  logic = &instance;
  logic->begin(millis());
#ifdef DEBUG_SERIAL
  DEBUG_SERIAL.print("POUK MCU v2 ready, session=");
  DEBUG_SERIAL.println(session);
#endif
}

void loop() {
  uint32_t t0 = micros();
  while (LINK_SERIAL.available() > 0) {
    uint8_t b = (uint8_t)LINK_SERIAL.read();
    logic->onByte(b, micros(), millis());
  }
  logic->tick(millis(), micros());
  logic->noteLoopTime(micros() - t0);
#ifdef DEBUG_SERIAL
  if (logic->state() != last_state) {           // смена состояния — в отладочный порт
    last_state = logic->state();
    DEBUG_SERIAL.print("state=");
    DEBUG_SERIAL.println(last_state);
  }
#endif
  // delay() запрещён: цикл обязан успевать за периодом телеметрии (бюджет — отчёт ЛР2).
}

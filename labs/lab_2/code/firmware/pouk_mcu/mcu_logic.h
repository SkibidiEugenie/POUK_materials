// ============================================================================
// Логика прошивки стенда (уровень 1): режимы, сторожевой таймер, команды,
// телеметрия. Переносимый C++98: доступ к шине приводов и к порту — через
// интерфейсы BusIface и LinkTx, поэтому логика проверяется модульными
// тестами на ПК без платы (host_test/), а в прошивке к ней подключаются
// Dynamixel2Arduino и Serial1 (pouk_mcu.ino). Это и есть разделение забот 4C.
// ============================================================================
#ifndef POUK_MCU_LOGIC_H
#define POUK_MCU_LOGIC_H

#include "pouk_proto.h"

namespace pouk {

// Шина приводов: углы в миллирадианах от вертикальной сборки.
class BusIface {
 public:
  virtual ~BusIface() {}
  virtual bool setTorque(bool on) = 0;
  virtual bool writeGoals(const int16_t q_mrad[N_JOINTS], uint16_t vmax_mrad_s) = 0;
  virtual bool readPositions(int16_t q_mrad[N_JOINTS]) = 0;
};

// Передача в канал к одноплатнику.
class LinkTx {
 public:
  virtual ~LinkTx() {}
  virtual void write(const uint8_t *data, size_t len) = 0;
};

struct McuConfig {
  uint16_t wd_timeout_ms;           // тайм-аут сторожевого таймера (расчёт — ЛР2, шаг 7)
  uint16_t telem_period_ms;         // период телеметрии
  uint16_t diag_period_ms;          // период диагностики
  uint16_t default_vmax;            // мрад/с, если в команде vmax = 0
  int16_t  q_min[N_JOINTS];         // пределы суставов, мрад
  int16_t  q_max[N_JOINTS];
};

void defaultConfig(McuConfig &cfg);

class McuLogic {
 public:
  McuLogic(BusIface &bus, LinkTx &tx, const McuConfig &cfg, uint8_t session);

  void begin(uint32_t now_ms);                              // старт в HOLD
  void onByte(uint8_t b, uint32_t now_us, uint32_t now_ms); // байт из канала
  void tick(uint32_t now_ms, uint32_t now_us);              // сторожевой таймер, телеметрия
  void noteLoopTime(uint32_t loop_us);                      // длительность итерации loop()

  uint8_t  state() const { return state_; }
  uint16_t wdTrips() const { return wd_trips_; }
  uint16_t staleCmd() const { return stale_cmd_; }
  const RxCounters &rxCounters() const { return rx_.counters; }

 private:
  void handle(const Packet &pkt, uint32_t now_ms);
  void onMode(uint16_t cmd_id, uint8_t mode, uint32_t now_ms);
  void onJoints(uint16_t cmd_id, const int16_t q[N_JOINTS], uint16_t vmax, uint32_t now_ms);
  void hold();
  void sendAck(uint16_t cmd_id, uint8_t result, uint8_t reason);
  void sendTelem(uint32_t now_ms);
  void sendDiag();
  void send(uint8_t type, const uint8_t *payload, uint8_t len);

  BusIface  &bus_;
  LinkTx    &tx_;
  McuConfig  cfg_;
  Receiver   rx_;

  uint8_t  session_;
  uint16_t seq_[3];                 // свой счётчик у каждого типа: ACK, TELEM, DIAG
  uint8_t  state_;
  uint8_t  flags_;

  bool     have_sbc_session_;
  uint8_t  sbc_session_;
  bool     have_last_cmd_;
  uint16_t last_cmd_id_;
  uint32_t last_cmd_ms_;
  uint16_t max_cmd_gap_ms_;

  int16_t  goal_[N_JOINTS];
  int16_t  present_[N_JOINTS];

  uint32_t next_telem_ms_;
  uint32_t next_diag_ms_;
  uint16_t stale_cmd_;
  uint16_t wd_trips_;
  uint16_t loop_max_us_;
  uint16_t bus_fail_;
};

}  // namespace pouk

#endif  // POUK_MCU_LOGIC_H

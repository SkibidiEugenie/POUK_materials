// ============================================================================
// Модульные тесты прошивки на ПК: протокол и логика без платы и приводов.
// Сборка и запуск (Linux/WSL/MSYS2), одной строкой:
//   g++ -std=c++98 -Wall -Wextra -pedantic -I../pouk_mcu test_host.cpp
//       ../pouk_mcu/pouk_proto.cpp ../pouk_mcu/mcu_logic.cpp -o test_host && ./test_host
// или скриптом: sh build_and_run.sh
// Кроме проверок печатает контрольные кадры в vectors.txt: тот же набор кадров
// строит Python-библиотека pouk_link (test_cross_vectors.py) — реализации
// протокола на двух языках обязаны совпадать побайтно.
// ============================================================================
#include <cstdio>
#include <cstring>
#include "pouk_proto.h"
#include "mcu_logic.h"

using namespace pouk;

static int g_checks = 0, g_fail = 0;
#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fail++; \
  std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)

// ---------------- заглушки шины и канала ------------------------------------
class FakeBus : public BusIface {
 public:
  int16_t pos[N_JOINTS];
  int16_t goal[N_JOINTS];
  int writes;
  bool torque;
  bool fail;
  FakeBus() : writes(0), torque(false), fail(false) {
    for (int j = 0; j < N_JOINTS; j++) { pos[j] = (int16_t)(100 * j); goal[j] = 0; }
  }
  bool setTorque(bool on) { torque = on; return !fail; }
  bool writeGoals(const int16_t q[N_JOINTS], uint16_t) {
    writes++;
    for (int j = 0; j < N_JOINTS; j++) goal[j] = q[j];
    return !fail;
  }
  bool readPositions(int16_t q[N_JOINTS]) {
    if (fail) return false;
    for (int j = 0; j < N_JOINTS; j++) q[j] = pos[j];
    return true;
  }
};

class CaptureTx : public LinkTx {
 public:
  Receiver rx;
  Packet last_ack, last_telem;
  int acks, telems, diags;
  CaptureTx() : rx(1000000UL), acks(0), telems(0), diags(0) {}
  void write(const uint8_t *data, size_t len) {
    Packet p;
    for (size_t i = 0; i < len; i++) {
      if (rx.feed(data[i], 0, p)) {
        if (p.type == T_ACK) { last_ack = p; acks++; }
        else if (p.type == T_TELEM) { last_telem = p; telems++; }
        else if (p.type == T_DIAG) { diags++; }
      }
    }
  }
};

static size_t frameJoints(uint8_t session, uint16_t seq, uint16_t cmd_id, const int16_t q[N_JOINTS],
                          uint16_t vmax, uint8_t *out) {
  uint8_t p[LEN_CMD_JOINTS];
  putU16(p, cmd_id);
  for (int j = 0; j < N_JOINTS; j++) putU16(p + 2 + 2 * j, (uint16_t)q[j]);
  putU16(p + 14, vmax);
  return buildFrame(T_CMD_JOINTS, session, seq, p, LEN_CMD_JOINTS, out);
}

static size_t frameMode(uint8_t session, uint16_t seq, uint16_t cmd_id, uint8_t mode, uint8_t *out) {
  uint8_t p[LEN_CMD_MODE];
  putU16(p, cmd_id);
  p[2] = mode;
  return buildFrame(T_CMD_MODE, session, seq, p, LEN_CMD_MODE, out);
}

static void feedAll(McuLogic &m, const uint8_t *f, size_t n, uint32_t now_ms) {
  for (size_t i = 0; i < n; i++) m.onByte(f[i], now_ms * 1000UL, now_ms);
}

static uint8_t ackResult(const CaptureTx &tx) { return tx.last_ack.payload[2]; }
static uint8_t ackReason(const CaptureTx &tx) { return tx.last_ack.payload[3]; }
static uint16_t ackId(const CaptureTx &tx) { return getU16(tx.last_ack.payload); }

// ---------------- тесты протокола -------------------------------------------
static void testCrcCobs() {
  const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc32(check, 9) == 0xCBF43926UL);

  uint8_t out[300], back[300];
  const uint8_t a[] = {0x00};
  CHECK(cobsEncode(a, 1, out) == 2 && out[0] == 1 && out[1] == 1);
  const uint8_t b[] = {0x11, 0x22, 0x00, 0x33};
  size_t n = cobsEncode(b, 4, out);
  CHECK(n == 5 && out[0] == 3 && out[1] == 0x11 && out[2] == 0x22 && out[3] == 2 && out[4] == 0x33);
  CHECK(cobsDecode(out, n, back) == 4 && std::memcmp(back, b, 4) == 0);
  uint8_t c[254];
  for (int i = 0; i < 254; i++) c[i] = (uint8_t)(i + 1);
  n = cobsEncode(c, 254, out);
  CHECK(n == 256 && out[0] == 0xFF);
  CHECK(cobsDecode(out, n, back) == 254 && std::memcmp(back, c, 254) == 0);
  const uint8_t bad[] = {0x03, 0x11, 0x00};
  CHECK(cobsDecode(bad, 3, back) == 0);
}

static void testReceiver() {
  uint8_t f[MAX_FRAME], f2[MAX_FRAME];
  int16_t q[N_JOINTS] = {1, -2, 300, -400, 5000, -6000};
  size_t n = frameJoints(7, 42, 1000, q, 900, f);
  Receiver rx(2000);
  Packet p;
  int got = 0;
  // мусор перед кадром: без разделителя он склеится с кадром и даст ошибку,
  // поэтому передатчик начинает работу с разделителя
  const uint8_t junk[] = {0x55, 0xAA, 0x00};
  for (int i = 0; i < 3; i++) got += rx.feed(junk[i], 100, p);
  CHECK(got == 0 && (rx.counters.cobs_err + rx.counters.len_err) == 1);
  for (size_t i = 0; i < n; i++) got += rx.feed(f[i], 200 + i * 87, p);
  CHECK(got == 1 && p.type == T_CMD_JOINTS && p.session == 7 && p.seq == 42 && p.len == LEN_CMD_JOINTS);
  CHECK(getU16(p.payload) == 1000 && getI16(p.payload + 2 + 2 * 5) == -6000);

  // искажение одного бита -> счётчик CRC (или длины/COBS, если задет служебный байт)
  std::memcpy(f2, f, n);
  f2[8] ^= 0x04;
  uint16_t crc0 = rx.counters.crc_err;
  got = 0;
  for (size_t i = 0; i < n; i++) got += rx.feed(f2[i], 10000 + i * 87, p);
  CHECK(got == 0 && rx.counters.crc_err == crc0 + 1);

  // обрыв кадра + пауза > тайм-аута -> timeout, следующий кадр принимается
  uint16_t to0 = rx.counters.timeout;
  for (size_t i = 0; i < 10; i++) rx.feed(f[i], 20000 + i * 87, p);
  got = 0;
  for (size_t i = 0; i < n; i++) got += rx.feed(f[i], 40000 + i * 87, p);
  CHECK(rx.counters.timeout == to0 + 1 && got == 1);

  // переполнение: 70 ненулевых байтов подряд, затем разделитель и нормальный кадр
  uint16_t of0 = rx.counters.overflow;
  for (int i = 0; i < 70; i++) rx.feed(0x42, 50000 + i * 87, p);
  CHECK(rx.state() == Receiver::DISCARD && rx.counters.overflow == of0 + 1);
  rx.feed(0x00, 57000, p);
  got = 0;
  for (size_t i = 0; i < n; i++) got += rx.feed(f[i], 58000 + i * 87, p);
  CHECK(got == 1);
}

// ---------------- тесты логики ----------------------------------------------
static void testLogic() {
  FakeBus bus;
  CaptureTx tx;
  McuConfig cfg;
  defaultConfig(cfg);
  cfg.wd_timeout_ms = 60;
  McuLogic m(bus, tx, cfg, 0x5A);
  uint8_t f[MAX_FRAME];
  uint32_t t = 1000;

  m.begin(t);
  CHECK(m.state() == ST_HOLD && bus.torque && bus.goal[3] == 300);     // удержание где стоим

  int16_t q[N_JOINTS] = {0, 100, 200, 300, 0, 0};
  size_t n = frameJoints(9, 0, 10, q, 0, f);
  feedAll(m, f, n, t);
  CHECK(tx.acks == 1 && ackResult(tx) == REJECTED && ackReason(tx) == R_MODE);   // не в RUN

  n = frameMode(9, 1, 11, MODE_RUN, f);
  feedAll(m, f, n, t);
  CHECK(m.state() == ST_RUN && ackResult(tx) == ACCEPTED && ackId(tx) == 11);

  int w0 = bus.writes;
  n = frameJoints(9, 2, 12, q, 0, f);
  feedAll(m, f, n, t + 5);
  CHECK(ackResult(tx) == ACCEPTED && bus.writes == w0 + 1 && bus.goal[2] == 200);

  // дубликат (повтор после потери ACK): подтверждается, в шину не пишется
  feedAll(m, f, n, t + 10);
  CHECK(ackResult(tx) == ACCEPTED && ackId(tx) == 12 && bus.writes == w0 + 1);

  // устаревшая команда
  n = frameJoints(9, 3, 11, q, 0, f);
  feedAll(m, f, n, t + 15);
  CHECK(ackResult(tx) == REJECTED && ackReason(tx) == R_STALE && m.staleCmd() == 1);

  // предел сустава: отказ без частичной записи
  int16_t qbad[N_JOINTS] = {0, 100, 200, 3000, 0, 0};
  n = frameJoints(9, 4, 13, qbad, 0, f);
  feedAll(m, f, n, t + 20);
  CHECK(ackResult(tx) == REJECTED && ackReason(tx) == R_LIMITS && bus.writes == w0 + 1);

  // сторожевой таймер: последняя принятая команда в t+10 (дубликат), T_wd = 60 мс
  m.tick(t + 60, (t + 60) * 1000UL);
  CHECK(m.state() == ST_RUN);
  m.tick(t + 71, (t + 71) * 1000UL);
  CHECK(m.state() == ST_WD_STOP && m.wdTrips() == 1);

  n = frameJoints(9, 5, 14, q, 0, f);
  feedAll(m, f, n, t + 80);
  CHECK(ackResult(tx) == REJECTED && ackReason(tx) == R_MODE);        // автоматического возврата нет

  n = frameMode(9, 6, 15, MODE_RUN, f);
  feedAll(m, f, n, t + 90);
  CHECK(m.state() == ST_RUN);

  n = frameMode(9, 7, 16, MODE_RELAX, f);
  feedAll(m, f, n, t + 95);
  CHECK(m.state() == ST_RELAX && !bus.torque);
  n = frameMode(9, 8, 17, MODE_RUN, f);
  feedAll(m, f, n, t + 96);
  CHECK(m.state() == ST_RELAX && ackResult(tx) == REJECTED && ackReason(tx) == R_MODE);

  // новая сессия верхнего уровня: нумерация команд сбрасывается
  n = frameMode(77, 0, 1, MODE_HOLD, f);
  feedAll(m, f, n, t + 100);
  CHECK(m.state() == ST_HOLD && ackResult(tx) == ACCEPTED);

  // телеметрия раз в 10 мс
  int tel0 = tx.telems;
  for (uint32_t k = 0; k <= 100; k++) m.tick(t + 200 + k, (t + 200 + k) * 1000UL);
  CHECK(tx.telems - tel0 >= 10 && tx.telems - tel0 <= 11);
  CHECK(tx.last_telem.payload[16] == ST_HOLD);
}

// ---------------- контрольные кадры для сверки с Python ---------------------
static void writeVectors() {
  std::FILE *fp = std::fopen("vectors.txt", "w");
  if (!fp) return;
  uint8_t f[MAX_FRAME];
  int16_t q[N_JOINTS] = {0, 612, -1278, 1943, -1790, 1023};
  size_t n = frameJoints(0x21, 0x1234, 0xBEEF, q, 800, f);
  std::fprintf(fp, "CMD_JOINTS ");
  for (size_t i = 0; i < n; i++) std::fprintf(fp, "%02X", f[i]);
  std::fprintf(fp, "\n");
  n = frameMode(0x21, 0x1235, 0xBEF0, MODE_RUN, f);
  std::fprintf(fp, "CMD_MODE ");
  for (size_t i = 0; i < n; i++) std::fprintf(fp, "%02X", f[i]);
  std::fprintf(fp, "\n");
  uint8_t p[LEN_TELEM];
  putU32(p, 123456789UL);
  for (int j = 0; j < N_JOINTS; j++) putU16(p + 4 + 2 * j, (uint16_t)q[j]);
  p[16] = ST_RUN;
  p[17] = F_MOVING | F_TORQUE_ON;
  putU16(p + 18, 0xBEEF);
  n = buildFrame(T_TELEM, 0x5A, 65535, p, LEN_TELEM, f);
  std::fprintf(fp, "TELEM ");
  for (size_t i = 0; i < n; i++) std::fprintf(fp, "%02X", f[i]);
  std::fprintf(fp, "\n");
  std::fclose(fp);
}

int main() {
  testCrcCobs();
  testReceiver();
  testLogic();
  writeVectors();
  std::printf("%d проверок, %d отказов\n", g_checks, g_fail);
  return g_fail == 0 ? 0 : 1;
}

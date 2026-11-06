// ============================================================================
// Протокол стенда v2: CRC-32, COBS, пакеты, автомат приёмника.
// Переносимый C++98 без зависимостей от Arduino: тот же файл собирается в
// прошивку OpenCM9.04 и в модульные тесты на ПК (host_test/).
// Спецификация — «Инструкция ЛР2», раздел «Протокол стенда v2».
// ============================================================================
#ifndef POUK_PROTO_H
#define POUK_PROTO_H

#include <stddef.h>
#include <stdint.h>

namespace pouk {

// ---------------- константы протокола ---------------------------------------
const uint8_t  N_JOINTS     = 6;
const size_t   HEADER_LEN   = 5;     // type, session, seq(2), len
const size_t   CRC_LEN      = 4;
const size_t   MIN_PACKET   = HEADER_LEN + CRC_LEN;
const size_t   MAX_PAYLOAD  = 24;    // пакет <= 33 байт: CRC-32 даёт HD = 6
const size_t   MAX_PACKET   = HEADER_LEN + MAX_PAYLOAD + CRC_LEN;
const size_t   MAX_ENCODED  = 64;    // буфер приёмника под кадр COBS
const size_t   MAX_FRAME    = MAX_PACKET + 2 + 1;   // COBS (+1 для <254 байт) + запас + 0x00

// типы пакетов
const uint8_t T_CMD_JOINTS = 0x01;   // cmd_id u16, q[6] i16 мрад, vmax u16 мрад/с   (16 байт)
const uint8_t T_CMD_MODE   = 0x02;   // cmd_id u16, mode u8                         (3 байта)
const uint8_t T_ACK        = 0x81;   // cmd_id u16, result u8, reason u8             (4 байта)
const uint8_t T_TELEM      = 0x82;   // t_ms u32, q[6] i16, state u8, flags u8, last_cmd u16 (20)
const uint8_t T_DIAG       = 0x83;   // rx_ok u32 + 10 x u16                          (24 байта)

const uint8_t LEN_CMD_JOINTS = 16, LEN_CMD_MODE = 3, LEN_ACK = 4, LEN_TELEM = 20, LEN_DIAG = 24;

// режимы, состояния, флаги, результаты
const uint8_t MODE_HOLD = 0, MODE_RUN = 1, MODE_RELAX = 2;
const uint8_t ST_BOOT = 0, ST_HOLD = 1, ST_RUN = 2, ST_WD_STOP = 3, ST_RELAX = 4, ST_FAULT = 5;
const uint8_t F_MOVING = 0x01, F_BUS_ERROR = 0x02, F_CMD_REJECTED = 0x04, F_TORQUE_ON = 0x08;
const uint8_t ACCEPTED = 0, REJECTED = 1;
const uint8_t R_OK = 0, R_LIMITS = 1, R_MODE = 2, R_STALE = 3, R_BAD_VALUE = 4;

// ---------------- CRC-32/ISO-HDLC и COBS ------------------------------------
uint32_t crc32(const uint8_t *data, size_t len);

// Возвращает длину результата. out должен вмещать len + len/254 + 1 байт.
size_t cobsEncode(const uint8_t *in, size_t len, uint8_t *out);
// Возвращает длину результата или 0 при нарушении кодировки.
size_t cobsDecode(const uint8_t *in, size_t len, uint8_t *out);

// ---------------- порядок байтов (little-endian) ----------------------------
inline void putU16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
inline void putU32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
inline uint16_t getU16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline int16_t  getI16(const uint8_t *p) { return (int16_t)getU16(p); }
inline uint32_t getU32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Собрать кадр на линии: COBS(пакет) + 0x00. Возвращает длину кадра.
size_t buildFrame(uint8_t type, uint8_t session, uint16_t seq,
                  const uint8_t *payload, uint8_t plen, uint8_t *frame);

// Ожидаемая длина полезной нагрузки для типа; 0 — тип неизвестен.
uint8_t expectedLen(uint8_t type);

// ---------------- принятый пакет и счётчики ---------------------------------
struct Packet {
  uint8_t  type;
  uint8_t  session;
  uint16_t seq;
  uint8_t  len;
  uint8_t  payload[MAX_PAYLOAD];
};

struct RxCounters {
  uint32_t rx_ok;
  uint16_t crc_err, cobs_err, len_err, timeout, overflow;
};

// ---------------- автомат приёмника -----------------------------------------
// Состояния WAIT / IN_FRAME / DISCARD; события: байт 0x00, прочий байт,
// межбайтовый тайм-аут. Полная функция переходов — в инструкции ЛР2.
class Receiver {
 public:
  enum State { WAIT = 0, IN_FRAME = 1, DISCARD = 2 };

  explicit Receiver(uint32_t interbyte_timeout_us = 2000);

  // Подать один байт. Возвращает true, если в pkt собран корректный пакет.
  bool feed(uint8_t b, uint32_t now_us, Packet &pkt);
  // Проверить тайм-аут без новых байтов (вызывать из цикла).
  void poll(uint32_t now_us);

  State state() const { return state_; }
  RxCounters counters;

 private:
  bool finishFrame(Packet &pkt);

  State    state_;
  uint8_t  buf_[MAX_ENCODED];
  size_t   len_;
  uint32_t last_us_;
  uint32_t timeout_us_;
};

}  // namespace pouk

#endif  // POUK_PROTO_H

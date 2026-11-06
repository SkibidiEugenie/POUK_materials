// Протокол стенда v2: реализация. См. pouk_proto.h.
#include "pouk_proto.h"

namespace pouk {

// ---------------- CRC-32/ISO-HDLC -------------------------------------------
// Полином 0x04C11DB7 в отражённой записи 0xEDB88320; init и xorout 0xFFFFFFFF.
// Таблица строится один раз при первом вызове: 256 записей, 1 КБ ОЗУ.
static uint32_t crc_table[256];
static bool crc_ready = false;

static void crcInit() {
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int k = 0; k < 8; k++) c = (c & 1u) ? (c >> 1) ^ 0xEDB88320UL : (c >> 1);
    crc_table[i] = c;
  }
  crc_ready = true;
}

uint32_t crc32(const uint8_t *data, size_t len) {
  if (!crc_ready) crcInit();
  uint32_t c = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) c = crc_table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
  return c ^ 0xFFFFFFFFUL;
}

// ---------------- COBS ------------------------------------------------------
size_t cobsEncode(const uint8_t *in, size_t len, uint8_t *out) {
  size_t read = 0, write = 1, code_pos = 0;
  uint8_t code = 1;
  while (read < len) {
    if (in[read] == 0) {
      out[code_pos] = code;
      code_pos = write++;
      code = 1;
    } else {
      out[write++] = in[read];
      if (++code == 0xFF) {
        out[code_pos] = code;
        code_pos = write++;
        code = 1;
      }
    }
    read++;
  }
  out[code_pos] = code;
  return write;
}

size_t cobsDecode(const uint8_t *in, size_t len, uint8_t *out) {
  size_t read = 0, write = 0;
  if (len == 0) return 0;
  while (read < len) {
    uint8_t code = in[read];
    if (code == 0) return 0;                       // нулевой байт внутри кадра
    if (read + code > len) return 0;               // код выходит за конец кадра
    read++;
    for (uint8_t i = 1; i < code; i++) {
      if (in[read] == 0) return 0;
      out[write++] = in[read++];
    }
    if (code < 0xFF && read < len) out[write++] = 0;
  }
  return write;
}

// ---------------- пакеты ----------------------------------------------------
uint8_t expectedLen(uint8_t type) {
  switch (type) {
    case T_CMD_JOINTS: return LEN_CMD_JOINTS;
    case T_CMD_MODE:   return LEN_CMD_MODE;
    case T_ACK:        return LEN_ACK;
    case T_TELEM:      return LEN_TELEM;
    case T_DIAG:       return LEN_DIAG;
    default:           return 0;
  }
}

size_t buildFrame(uint8_t type, uint8_t session, uint16_t seq,
                  const uint8_t *payload, uint8_t plen, uint8_t *frame) {
  uint8_t pkt[MAX_PACKET];
  if (plen > MAX_PAYLOAD) return 0;
  pkt[0] = type;
  pkt[1] = session;
  putU16(pkt + 2, seq);
  pkt[4] = plen;
  for (uint8_t i = 0; i < plen; i++) pkt[HEADER_LEN + i] = payload[i];
  putU32(pkt + HEADER_LEN + plen, crc32(pkt, HEADER_LEN + plen));
  size_t n = cobsEncode(pkt, HEADER_LEN + plen + CRC_LEN, frame);
  frame[n++] = 0x00;
  return n;
}

// ---------------- автомат приёмника -----------------------------------------
Receiver::Receiver(uint32_t interbyte_timeout_us)
    : state_(WAIT), len_(0), last_us_(0), timeout_us_(interbyte_timeout_us) {
  counters.rx_ok = 0;
  counters.crc_err = counters.cobs_err = counters.len_err = 0;
  counters.timeout = counters.overflow = 0;
}

void Receiver::poll(uint32_t now_us) {
  if (state_ != WAIT && (uint32_t)(now_us - last_us_) > timeout_us_) {
    if (state_ == IN_FRAME) counters.timeout++;
    state_ = WAIT;
    len_ = 0;
  }
}

bool Receiver::finishFrame(Packet &pkt) {
  uint8_t raw[MAX_ENCODED];
  size_t n = cobsDecode(buf_, len_, raw);
  if (n == 0) { counters.cobs_err++; return false; }
  if (n < MIN_PACKET || n != MIN_PACKET + raw[4]) { counters.len_err++; return false; }
  uint8_t plen = raw[4];
  if (crc32(raw, HEADER_LEN + plen) != getU32(raw + HEADER_LEN + plen)) {
    counters.crc_err++;
    return false;
  }
  if (expectedLen(raw[0]) != plen) { counters.len_err++; return false; }
  pkt.type = raw[0];
  pkt.session = raw[1];
  pkt.seq = getU16(raw + 2);
  pkt.len = plen;
  for (uint8_t i = 0; i < plen; i++) pkt.payload[i] = raw[HEADER_LEN + i];
  counters.rx_ok++;
  return true;
}

bool Receiver::feed(uint8_t b, uint32_t now_us, Packet &pkt) {
  bool got = false;
  poll(now_us);                                   // событие «тайм-аут» — первым
  if (b == 0x00) {
    if (state_ == IN_FRAME) got = finishFrame(pkt);
    state_ = WAIT;                                // в WAIT и DISCARD разделитель — начало нового кадра
    len_ = 0;
  } else if (state_ == WAIT) {
    buf_[0] = b;
    len_ = 1;
    state_ = IN_FRAME;
  } else if (state_ == IN_FRAME) {
    if (len_ >= MAX_ENCODED) {
      counters.overflow++;
      state_ = DISCARD;
      len_ = 0;
    } else {
      buf_[len_++] = b;
    }
  }                                               // DISCARD: байт пропускается
  last_us_ = now_us;
  return got;
}

}  // namespace pouk

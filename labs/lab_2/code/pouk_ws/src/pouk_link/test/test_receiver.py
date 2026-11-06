# -*- coding: utf-8 -*-
"""ЛР2, шаг 4: автомат приёмника. Каждый тест — строка таблицы переходов из инструкции."""
from pouk_link import protocol as P

R = P.Receiver


def frame(seq=1, cmd_id=10):
    return P.build_frame(P.T_CMD_MODE, 0x33, seq, P.pack_cmd_mode(cmd_id, P.MODE_HOLD))


def feed_bytes(rx, data, t0=0.0, dt=0.0001):
    out = []
    for i, b in enumerate(data):
        out.extend(rx.feed(bytes([b]), t0 + i * dt))
    return out


def test_wait_zero_is_ignored():                     # WAIT, 0x00 -> WAIT, ничего не выдаётся
    rx = R()
    assert rx.feed(b'\x00\x00\x00', 0.0) == []
    assert rx.state == R.WAIT


def test_wait_byte_starts_frame():                   # WAIT, байт -> IN_FRAME
    rx = R()
    assert rx.feed(b'\x05', 0.0) == []
    assert rx.state == R.IN_FRAME


def test_in_frame_zero_completes_packet():           # IN_FRAME, 0x00 -> пакет, WAIT
    rx = R()
    out = feed_bytes(rx, frame())
    assert len(out) == 1 and isinstance(out[0], P.Packet)
    assert out[0].type == P.T_CMD_MODE and out[0].seq == 1
    assert rx.state == R.WAIT


def test_in_frame_zero_bad_crc():                    # IN_FRAME, 0x00, CRC не сошлась -> crc_err
    f = bytearray(frame())
    f[2] ^= 0x01                                     # байт данных внутри кадра (поле session)
    rx = R()
    assert feed_bytes(rx, f) == [P.RxError.CRC]
    assert rx.state == R.WAIT


def test_in_frame_zero_bad_cobs():                   # IN_FRAME, 0x00, кодировка нарушена -> cobs_err
    rx = R()
    assert rx.feed(b'\x09\x01\x02\x00', 0.0) == [P.RxError.COBS]


def test_in_frame_zero_short_packet():               # IN_FRAME, 0x00, пакет короче минимума -> len_err
    rx = R()
    assert rx.feed(b'\x02\x01\x00', 0.0) == [P.RxError.LENGTH]


def test_unknown_type():                             # корректная CRC, но неизвестный тип -> type_err
    rx = R()
    f = P.build_frame(0x7E, 1, 1, b'\x01\x02\x03')
    assert rx.feed(f, 0.0) == [P.RxError.TYPE]


def test_in_frame_overflow_then_discard():           # IN_FRAME, переполнение -> DISCARD
    rx = R(max_encoded=64)
    out = rx.feed(b'\x42' * 70, 0.0)
    assert out == [P.RxError.OVERFLOW]
    assert rx.state == R.DISCARD
    assert rx.feed(b'\x42' * 10, 0.0) == []           # DISCARD, байт -> DISCARD
    assert rx.feed(b'\x00', 0.0) == []                # DISCARD, 0x00 -> WAIT без выдачи
    assert rx.state == R.WAIT
    assert len(rx.feed(frame(), 0.0)) == 1            # и приём продолжается


def test_in_frame_timeout_resync():                  # IN_FRAME, тайм-аут -> timeout, WAIT
    rx = R(interbyte_timeout=0.002)
    f = frame()
    assert rx.feed(f[:5], 0.000) == []
    out = rx.feed(f, 0.010)                           # пауза 10 мс, затем целый кадр
    assert out[0] == P.RxError.TIMEOUT
    assert len(out) == 2 and isinstance(out[1], P.Packet)


def test_poll_reports_timeout_once():
    rx = R(interbyte_timeout=0.002)
    rx.feed(frame()[:4], 0.0)
    assert rx.poll(0.001) == []
    assert rx.poll(0.005) == [P.RxError.TIMEOUT]
    assert rx.poll(0.010) == []
    assert rx.state == R.WAIT


def test_garbage_between_frames_costs_one_error():
    rx = R()
    stream = frame(1) + b'\x13\x37\xff' + frame(2) + frame(3)
    out = rx.feed(stream, 0.0)
    errors = [o for o in out if isinstance(o, str)]
    packets = [o for o in out if isinstance(o, P.Packet)]
    assert len(errors) == 1                            # мусор склеился с кадром 2 — один отказ
    assert [p.seq for p in packets] == [1, 3]


def test_split_frame_across_reads():
    rx = R()
    f = frame(7)
    out = []
    for i in range(0, len(f), 3):
        out.extend(rx.feed(f[i:i + 3], i * 0.0001))
    assert len(out) == 1 and out[0].seq == 7

# -*- coding: utf-8 -*-
"""ЛР2: CRC-32, COBS, пакеты протокола стенда v2. Запуск: python3 -m pytest test/"""
import os
import random

import pytest

from pouk_link import cobs
from pouk_link import protocol as P


def test_crc32_check_value():
    # контрольное значение CRC-32/ISO-HDLC для строки «123456789»
    assert P.crc32(b'123456789') == 0xCBF43926


@pytest.mark.parametrize('raw, enc', [
    (b'\x00', b'\x01\x01'),
    (b'\x00\x00', b'\x01\x01\x01'),
    (b'\x11\x22\x00\x33', b'\x03\x11\x22\x02\x33'),
    (b'\x11\x22\x33\x44', b'\x05\x11\x22\x33\x44'),
    (b'\x11\x00\x00\x00', b'\x02\x11\x01\x01\x01'),
    # алгоритм статьи Cheshire–Baker: блок из 254 ненулевых байтов в конце данных
    # закрывается кодом 0x01 (декодер понимает и минимальную форму без него)
    (bytes(range(1, 255)), b'\xff' + bytes(range(1, 255)) + b'\x01'),
    (bytes(range(0, 255)), b'\x01\xff' + bytes(range(1, 255)) + b'\x01'),
])
def test_cobs_vectors(raw, enc):
    assert cobs.encode(raw) == enc
    assert cobs.decode(enc) == raw


def test_cobs_decodes_minimal_form():
    assert cobs.decode(b'\xff' + bytes(range(1, 255))) == bytes(range(1, 255))


def test_cobs_roundtrip_random():
    rng = random.Random(5)
    for _ in range(500):
        data = bytes(rng.randrange(256) for _ in range(rng.randrange(0, 600)))
        e = cobs.encode(data)
        assert 0 not in e
        assert cobs.decode(e) == data


def test_cobs_rejects_zero_and_overrun():
    with pytest.raises(cobs.CobsError):
        cobs.decode(b'\x03\x11\x00')
    with pytest.raises(cobs.CobsError):
        cobs.decode(b'\x05\x11')


def test_packet_sizes_keep_hamming_distance_6():
    # пакет под CRC (без самой CRC) не длиннее 268 бит — HD = 6 для CRC-32 (пособие, рис. 2.5)
    for ptype, st in P.PAYLOAD.items():
        assert (P.HEADER_LEN + st.size) * 8 <= 268, P.TYPE_NAMES[ptype]


def test_frame_roundtrip_all_types():
    q = [0.0, 0.612, -1.278, 1.943, -1.790, 1.023]
    cases = [
        (P.T_CMD_JOINTS, P.pack_cmd_joints(0xBEEF, q, 0.8)),
        (P.T_CMD_MODE, P.pack_cmd_mode(7, P.MODE_RUN)),
        (P.T_ACK, P.pack_ack(7, P.REJECTED, P.R_LIMITS)),
        (P.T_TELEM, P.pack_telem(123456789, q, P.ST_RUN, P.F_MOVING, 0xBEEF)),
        (P.T_DIAG, P.pack_diag((1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0))),
    ]
    rx = P.Receiver()
    stream = b''.join(P.build_frame(t, 0x21, i, pl) for i, (t, pl) in enumerate(cases))
    out = rx.feed(stream, 0.0)
    assert [p.type for p in out] == [t for t, _ in cases]
    j = P.unpack(out[0])
    assert j.cmd_id == 0xBEEF and j.q == q and abs(j.vmax - 0.8) < 1e-9
    t = P.unpack(out[3])
    assert t.t_ms == 123456789 and t.state == P.ST_RUN and t.last_cmd == 0xBEEF


def test_seq_distance_wraps():
    assert P.seq_distance(0, 65535) == 1
    assert P.seq_distance(5, 5) == 0
    assert P.seq_distance(3, 10) > 32768          # «старше»


def test_units_saturate():
    assert P.rad_to_mrad(1.0004) == 1000
    assert P.rad_to_mrad(-0.0016) == -2
    assert P.rad_to_mrad(100.0) == 32767
    assert P.rad_to_mrad(-100.0) == -32768


VECTORS = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'firmware', 'host_test',
                       'vectors.txt')


@pytest.mark.skipif(not os.path.exists(VECTORS), reason='нет vectors.txt: соберите и запустите host_test')
def test_cross_check_with_firmware():
    """Python и C++ реализации протокола строят побайтно одинаковые кадры."""
    q = [0.0, 0.612, -1.278, 1.943, -1.790, 1.023]
    ours = {
        'CMD_JOINTS': P.build_frame(P.T_CMD_JOINTS, 0x21, 0x1234, P.pack_cmd_joints(0xBEEF, q, 0.8)),
        'CMD_MODE': P.build_frame(P.T_CMD_MODE, 0x21, 0x1235, P.pack_cmd_mode(0xBEF0, P.MODE_RUN)),
        'TELEM': P.build_frame(P.T_TELEM, 0x5A, 65535,
                               P.pack_telem(123456789, q, P.ST_RUN, P.F_MOVING | P.F_TORQUE_ON, 0xBEEF)),
    }
    with open(VECTORS) as f:
        theirs = dict(line.split() for line in f if line.strip())
    for name, frame in ours.items():
        assert frame.hex().upper() == theirs[name], name

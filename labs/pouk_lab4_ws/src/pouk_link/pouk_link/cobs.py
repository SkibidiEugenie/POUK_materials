# -*- coding: utf-8 -*-
"""COBS — Consistent Overhead Byte Stuffing (Cheshire, Baker, 1999).

Кадрирование протокола стенда v2 (пособие, §2.4.2). После кодирования в данных
нет ни одного нулевого байта, поэтому 0x00 однозначно отмечает конец кадра.
Накладные расходы: один байт на каждые 254 байта данных плюс один байт —
для пакетов протокола стенда (до 33 байт) ровно один байт.
"""


class CobsError(ValueError):
    """Нарушение кодировки COBS: нулевой байт внутри или код выходит за конец."""


def encode(data):
    """Закодировать bytes -> bytes без нулевых байтов (разделитель не добавляется)."""
    out = bytearray()
    code_pos = 0
    out.append(0)          # место под первый код
    code = 1
    for b in bytes(data):
        if b == 0:
            out[code_pos] = code
            code_pos = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_pos] = code
                code_pos = len(out)
                out.append(0)
                code = 1
    out[code_pos] = code
    return bytes(out)


def decode(data):
    """Раскодировать кадр без разделителя. Нарушение кодировки -> CobsError."""
    data = bytes(data)
    out = bytearray()
    i = 0
    n = len(data)
    if n == 0:
        raise CobsError('пустой кадр')
    while i < n:
        code = data[i]
        if code == 0:
            raise CobsError('нулевой байт внутри кадра')
        i += 1
        end = i + code - 1
        if end > n:
            raise CobsError('код COBS выходит за конец кадра')
        chunk = data[i:end]
        if 0 in chunk:
            raise CobsError('нулевой байт внутри кадра')
        out.extend(chunk)
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)

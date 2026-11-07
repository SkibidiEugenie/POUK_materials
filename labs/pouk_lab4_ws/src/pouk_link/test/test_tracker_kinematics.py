# -*- coding: utf-8 -*-
"""ЛР2, шаг 5: учёт команд; кинематика стенда (ядро без промежуточного ПО)."""
import math
import threading

from pouk_link import kinematics as K
from pouk_link import protocol as P
from pouk_link.link import CommandTracker


def ack(cmd_id, result=P.ACCEPTED, reason=P.R_OK):
    return P.Ack(cmd_id, result, reason)


def test_ack_completes_command_and_measures_rtt():
    tr = CommandTracker(ack_timeout=0.008, max_attempts=2)
    ev = threading.Event()
    tr.register(5, b'frame', 1.000, ev)
    res = tr.on_ack(ack(5), 1.003)
    assert res.ok and res.attempts == 1 and abs(res.rtt - 0.003) < 1e-9
    assert ev.is_set() and tr.take_result(5) == res
    assert tr.counters['cmd_acked'] == 1 and 5 not in tr.pending


def test_rejected_command_is_counted():
    tr = CommandTracker()
    tr.register(6, b'f', 0.0)
    res = tr.on_ack(ack(6, P.REJECTED, P.R_LIMITS), 0.002)
    assert not res.ok and res.reason == P.R_LIMITS
    assert tr.counters['cmd_rejected'] == 1


def test_unexpected_ack():
    tr = CommandTracker()
    assert tr.on_ack(ack(99), 0.0) is None
    assert tr.counters['ack_unexpected'] == 1


def test_single_retry_then_lost():
    tr = CommandTracker(ack_timeout=0.008, max_attempts=2)
    ev = threading.Event()
    tr.register(7, b'F7', 0.0, ev)
    assert tr.due_retries(0.005) == []                   # ещё рано
    assert tr.due_retries(0.009) == [b'F7']              # первый тайм-аут — тот же кадр
    assert tr.counters['cmd_retries'] == 1
    assert tr.due_retries(0.012) == []                   # новый срок отсчитывается от повтора
    assert tr.due_retries(0.018) == []                   # попытки исчерпаны — потеряна
    assert tr.counters['cmd_lost'] == 1 and ev.is_set()
    assert tr.take_result(7).ok is False


def test_rtt_counts_from_first_send_after_retry():
    tr = CommandTracker(ack_timeout=0.008, max_attempts=2)
    tr.register(8, b'F8', 0.0)
    tr.due_retries(0.009)
    res = tr.on_ack(ack(8), 0.011)
    assert res.attempts == 2 and abs(res.rtt - 0.011) < 1e-9


# ------------------------------------------------------------------ кинематика
def test_ik_fk_roundtrip_grid():
    for x in (0.06, 0.10, 0.14):
        for y in (-0.05, 0.0, 0.05):
            for z in (0.0, 0.03, 0.06):
                r = K.ik(x, y, z)
                if not r.ok:
                    continue
                fx, fy, fz = K.fk(r.q)
                assert math.dist((x, y, z), (fx, fy, fz)) < 1e-9
                assert abs(r.q[1] + r.q[2] + r.q[3] - math.pi) < 1e-9     # схват вертикален


def test_ik_rejects_unreachable_without_nan():
    r = K.ik(0.4, 0.0, 0.03)
    assert not r.ok and r.q is None and 'дальше' in r.reason


def test_default_circle_is_feasible_and_sagitta():
    pts = K.circle_points(0.100, 0.0, 0.030, 0.030, 20)
    assert all(K.ik(*p).ok for p in pts)
    assert abs(K.sagitta(30.0, 20) - 0.369) < 0.001


def test_limits_match_firmware_ticks():
    assert K.rad_to_ticks(K.JOINT_MIN[3]) == 362 and K.rad_to_ticks(K.JOINT_MAX[2]) == 892
    assert P.rad_to_mrad(K.JOINT_MAX[0]) == 1790 and P.rad_to_mrad(K.JOINT_MIN[3]) == -767
    for j in range(K.N_JOINTS):
        # целые миллирадианы выведены из отсчётов каркаса ЛР1 — и совпадают с прошивкой
        assert int(round(K.ticks_to_rad(K.JOINT_MIN_TICKS[j]) * 1000)) == K.JOINT_MIN_MRAD[j]
        assert int(round(K.ticks_to_rad(K.JOINT_MAX_TICKS[j]) * 1000)) == K.JOINT_MAX_MRAD[j]
    assert K.in_limits([K.JOINT_MAX[j] for j in range(K.N_JOINTS)])
    assert not K.in_limits([0.0, 0.0, 0.0, K.JOINT_MIN[3] - 0.001, 0.0, 0.0])


def test_limits_identical_in_firmware_source():
    import os
    import re
    src = os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'firmware', 'pouk_mcu',
                       'mcu_logic.cpp')
    if not os.path.exists(src):
        return                                   # пакет собран отдельно от прошивки
    text = open(src, encoding='utf-8').read()
    qmin = [int(v) for v in re.search(r'qmin\[N_JOINTS\] = \{([^}]*)\}', text).group(1).split(',')]
    qmax = [int(v) for v in re.search(r'qmax\[N_JOINTS\] = \{([^}]*)\}', text).group(1).split(',')]
    assert tuple(qmin) == K.JOINT_MIN_MRAD and tuple(qmax) == K.JOINT_MAX_MRAD

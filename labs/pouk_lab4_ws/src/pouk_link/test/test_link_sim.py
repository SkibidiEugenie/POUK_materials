# -*- coding: utf-8 -*-
"""Сквозная проверка ЛР2 на виртуальном стенде: канал, режимы, сторожевой таймер, помехи."""
import random
import socket
import threading
import time

import pytest

from pouk_link import kinematics as K
from pouk_link import protocol as P
from pouk_link import sim_mcu
from pouk_link.link import FaultInjector, StandLink

serial = pytest.importorskip('serial')


def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def stand():
    port = free_port()
    sim = sim_mcu.SimMcu(wd_ms=60, rng=random.Random(1))
    tr = sim_mcu.TcpTransport('127.0.0.1', port)
    stop = {'flag': False}

    def loop():
        t_prev = time.monotonic()
        while not stop['flag']:
            now = time.monotonic()
            data = tr.read()
            out = sim.feed(data, now) if data else []
            out.extend(sim.tick(now, now - t_prev))
            t_prev = now
            for f in out:
                tr.write(f)
            time.sleep(0.0005)

    th = threading.Thread(target=loop, daemon=True)
    th.start()
    yield sim, 'socket://127.0.0.1:%d' % port
    stop['flag'] = True
    th.join(1.0)


def test_telemetry_and_modes(stand):
    sim, url = stand
    with StandLink(url) as link:
        s = link.wait_telemetry(3.0)
        assert s is not None and s.state == P.ST_HOLD
        r = link.send_joints([0.0] * 6)
        assert r.ok is False and r.reason == P.R_MODE          # не в RUN — отказ
        link.set_target(s.q)
        link.stream_start(50)
        assert link.set_mode('run').ok
        target = K.with_gripper(K.ik(0.10, 0.0, 0.03).q)
        link.set_target(target, 1.5)
        assert link.wait_reached(target, 0.08, 5.0)
        st = link.stats()
        # под нагрузкой процессора возможна единичная потеря подтверждения (тайм-аут 8 мс × 2 попытки)
        assert st['telem_ok'] > 50 and st['cmd_lost'] <= max(1, 0.02 * st['cmd_sent']) and st['crc_err'] == 0
        assert st['telem_lost'] == 0                            # номера телеметрии идут подряд
        assert len(link.tracker.rtt) > 20


def test_watchdog_trips_when_stream_stops(stand):
    sim, url = stand
    with StandLink(url) as link:
        s = link.wait_telemetry(3.0)
        link.set_target(s.q)
        link.stream_start(50)
        assert link.set_mode('run').ok
        time.sleep(0.3)
        assert link.latest().state == P.ST_RUN
        link.stream_stop()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0 and link.latest().state != P.ST_WD_STOP:
            time.sleep(0.005)
        assert link.latest().state == P.ST_WD_STOP
        assert sim.cnt['wd_trips'] == 1
        r = link.send_joints(s.q)
        assert not r.ok and r.reason == P.R_MODE                # автоматического возврата нет


def test_limits_are_rejected(stand):
    sim, url = stand
    with StandLink(url) as link:
        s = link.wait_telemetry(3.0)
        link.set_target(s.q)
        link.stream_start(50)
        assert link.set_mode('run').ok
        q = list(s.q)
        q[3] = 2.5                                             # за пределом сустава 4
        r = link.send_joints(q)
        assert not r.ok and r.reason == P.R_LIMITS


def test_injected_corruption_is_diagnosed(stand):
    sim, url = stand
    fault = FaultInjector(drop=0.02, ber=2e-3, seed=3)
    with StandLink(url, fault=fault) as link:
        s = link.wait_telemetry(3.0)
        link.set_target(s.q)
        link.stream_start(50)
        link.set_mode('run')
        time.sleep(3.0)
        st = link.stats()
        mcu_errors = sim.cnt['crc_err'] + sim.cnt['cobs_err'] + sim.cnt['len_err'] + sim.cnt['timeout']
        # искажения, внесённые одноплатником, видны в счётчиках МК; потери — в повторах
        assert st['inj_corrupt'] > 0 and mcu_errors > 0
        assert st['cmd_retries'] > 0
        assert st['cmd_acked'] + st['cmd_rejected'] > 0.7 * st['cmd_sent']

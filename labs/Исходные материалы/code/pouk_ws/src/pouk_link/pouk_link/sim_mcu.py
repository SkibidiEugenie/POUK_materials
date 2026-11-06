# -*- coding: utf-8 -*-
"""Виртуальный стенд: программная модель микроконтроллера и приводов.

Говорит протоколом стенда v2 так же, как прошивка pouk_mcu: режимы, сторожевой
таймер, дедупликация команд, пределы суставов, телеметрия 100 Гц, диагностика
1 Гц. Приводы — модель с ограничением скорости, податливостью под нагрузкой,
квантованием датчика и шумом. Подменяет нижний уровень при неизменном
интерфейсе (пособие, §7.6.4): мост, CLI и узлы ROS 2 работают с ним без правок.

Запуск:
    python3 -m pouk_link.sim_mcu                       # TCP 127.0.0.1:5760
    python3 -m pouk_link.sim_mcu --pty                 # Linux: псевдотерминал /dev/pts/N
    python3 -m pouk_link.sim_mcu --stuck 2@8.0         # сустав 2 «заклинит» через 8 с
Порт для клиента: socket://127.0.0.1:5760 или путь псевдотерминала.
"""
import argparse
import math
import os
import random
import select
import socket
import sys
import time

from . import cobs
from . import kinematics as K
from . import protocol as P


class Framer(object):
    """Разбор входящих кадров для модели МК: разделение по 0x00 и проверка пакета.

    Намеренно проще автомата приёмника ЛР2 (нет межбайтового тайм-аута и режима
    пропуска): модель стенда не должна зависеть от кода, который пишут студенты, —
    иначе ошибка в их приёмнике сломает и эталон, с которым они сравнивают.
    """

    def __init__(self, max_encoded=P.MAX_ENCODED):
        self.max_encoded = max_encoded
        self.buf = bytearray()

    def feed(self, data, now=None):
        out = []
        for b in bytes(data):
            if b != 0:
                self.buf.append(b)
                continue
            if self.buf:
                if len(self.buf) > self.max_encoded:
                    out.append(P.RxError.OVERFLOW)
                else:
                    try:
                        raw = cobs.decode(self.buf)
                    except cobs.CobsError:
                        out.append(P.RxError.COBS)
                    else:
                        pkt, err = P.parse_packet(raw)
                        out.append(pkt if err is None else err)
            self.buf = bytearray()
        return out


class ServoModel(object):
    """Шесть приводов AX-12A: положение, ограничение скорости, податливость, датчик."""

    DEFAULT_SPEED = 1.2          # рад/с, если vmax = 0
    MAX_SPEED = 5.0              # рад/с, предел модели
    SAG = (0.0, 0.020, 0.012, 0.004, 0.0, 0.0)   # рад: статическая ошибка под нагрузкой

    def __init__(self, rng, noise=True):
        self.rng = rng
        self.noise = noise
        self.pos = [0.0] * P.N_JOINTS
        self.goal = [0.0] * P.N_JOINTS
        self.speed = [self.DEFAULT_SPEED] * P.N_JOINTS
        self.torque = False
        self.stuck = {}                              # сустав -> время заклинивания

    def set_goal(self, q, vmax):
        v = self.DEFAULT_SPEED if vmax <= 0 else min(vmax, self.MAX_SPEED)
        self.goal = list(q)
        self.speed = [v] * P.N_JOINTS
        self.torque = True

    def hold(self):
        self.goal = list(self.pos)
        self.torque = True

    def relax(self):
        self.torque = False

    def step(self, dt, t):
        for j in range(P.N_JOINTS):
            if j in self.stuck and t >= self.stuck[j]:
                continue
            if self.torque:
                # податливость: под действием веса привод не доходит до уставки
                load = math.sin(sum(self.pos[1:j + 1])) if 1 <= j <= 3 else 0.0
                target = self.goal[j] - self.SAG[j] * load
                err = target - self.pos[j]
                lim = self.speed[j] * dt
                self.pos[j] += max(-lim, min(lim, err))
            elif 1 <= j <= 3:
                # без момента звенья «оседают» под весом (медленно, модель)
                self.pos[j] += 0.05 * dt * math.copysign(1.0, self.pos[j] or 1.0)

    def measure(self):
        out = []
        for j in range(P.N_JOINTS):
            ticks = K.rad_to_ticks(self.pos[j])
            if self.noise and self.rng.random() < 0.1:
                ticks += self.rng.choice((-1, 1))
            out.append(K.ticks_to_rad(ticks))
        return out

    def moving(self):
        return any(abs(self.goal[j] - self.pos[j]) > 2 * K.DEL for j in range(P.N_JOINTS))


class SimMcu(object):
    """Логика прошивки pouk_mcu. Время — в секундах монотонных часов хоста."""

    def __init__(self, wd_ms=60, telem_hz=100.0, rng=None, noise=True):
        self.rng = rng or random.Random(7)
        self.servo = ServoModel(self.rng, noise)
        self.wd_s = wd_ms / 1000.0
        self.telem_period = 1.0 / telem_hz
        self.session = self.rng.randrange(1, 256)
        self.seq = {}                                # номер пакета — по каждому типу
        self.state = P.ST_HOLD
        self.flags = 0
        self.t0 = time.monotonic()
        self.sbc_session = None
        self.last_cmd_id = None
        self.last_cmd_t = None
        self.max_cmd_gap = 0.0
        self.rx = Framer()
        self.cnt = dict(rx_ok=0, crc_err=0, cobs_err=0, len_err=0, timeout=0, overflow=0,
                        stale_cmd=0, wd_trips=0)
        self.next_telem = 0.0
        self.next_diag = 0.0
        self.servo.hold()
        self.flags |= P.F_TORQUE_ON

    # ---------------------------------------------------------------- выдача
    def _frame(self, ptype, payload):
        seq = self.seq.get(ptype, 0)
        self.seq[ptype] = (seq + 1) & 0xFFFF
        return P.build_frame(ptype, self.session, seq, payload)

    def _ack(self, cmd_id, result, reason):
        if result == P.REJECTED:
            self.flags |= P.F_CMD_REJECTED
        else:
            self.flags &= ~P.F_CMD_REJECTED
        return self._frame(P.T_ACK, P.pack_ack(cmd_id, result, reason))

    # ---------------------------------------------------------------- приём
    def feed(self, data, now):
        out = []
        for r in self.rx.feed(data, now):
            if isinstance(r, str):
                key = 'len_err' if r == P.RxError.TYPE else r
                self.cnt[key] = self.cnt.get(key, 0) + 1
                continue
            self.cnt['rx_ok'] += 1
            out.extend(self._handle(r, now))
        return out

    def _handle(self, pkt, now):
        if pkt.type not in (P.T_CMD_JOINTS, P.T_CMD_MODE):
            self.cnt['len_err'] += 1
            return []
        cmd = P.unpack(pkt)
        if pkt.session != self.sbc_session:          # новая сессия верхнего уровня
            self.sbc_session = pkt.session
            self.last_cmd_id = None
        if self.last_cmd_id is not None:
            d = P.seq_distance(cmd.cmd_id, self.last_cmd_id)
            if d == 0:                               # дубликат: подтвердить повторно
                self.last_cmd_t = now
                return [self._ack(cmd.cmd_id, P.ACCEPTED, P.R_OK)]
            if d > 32768:                            # старше последней принятой
                self.cnt['stale_cmd'] += 1
                return [self._ack(cmd.cmd_id, P.REJECTED, P.R_STALE)]
        if pkt.type == P.T_CMD_MODE:
            return [self._on_mode(cmd, now)]
        return [self._on_joints(cmd, now)]

    def _on_mode(self, cmd, now):
        m = cmd.mode
        if m == P.MODE_HOLD:
            self.servo.hold()
            self.state = P.ST_HOLD
        elif m == P.MODE_RUN:
            if self.state == P.ST_RELAX:
                return self._ack(cmd.cmd_id, P.REJECTED, P.R_MODE)   # сначала HOLD
            if self.state != P.ST_RUN:
                self.servo.hold()                    # старт движения — от текущего положения
            self.state = P.ST_RUN
        elif m == P.MODE_RELAX:
            self.servo.relax()
            self.state = P.ST_RELAX
        else:
            return self._ack(cmd.cmd_id, P.REJECTED, P.R_BAD_VALUE)
        self.last_cmd_id = cmd.cmd_id
        self.last_cmd_t = now
        return self._ack(cmd.cmd_id, P.ACCEPTED, P.R_OK)

    def _on_joints(self, cmd, now):
        if self.state != P.ST_RUN:
            return self._ack(cmd.cmd_id, P.REJECTED, P.R_MODE)
        if not K.in_limits(cmd.q):
            return self._ack(cmd.cmd_id, P.REJECTED, P.R_LIMITS)
        self.servo.set_goal(cmd.q, cmd.vmax)
        if self.last_cmd_t is not None:
            self.max_cmd_gap = max(self.max_cmd_gap, now - self.last_cmd_t)
        self.last_cmd_id = cmd.cmd_id
        self.last_cmd_t = now
        return self._ack(cmd.cmd_id, P.ACCEPTED, P.R_OK)

    # ---------------------------------------------------------------- цикл
    def tick(self, now, dt):
        out = []
        self.servo.step(dt, now - self.t0)
        if self.state == P.ST_RUN and self.last_cmd_t is not None and now - self.last_cmd_t > self.wd_s:
            self.servo.hold()
            self.state = P.ST_WD_STOP
            self.cnt['wd_trips'] += 1
        self.flags = (self.flags & ~(P.F_MOVING | P.F_TORQUE_ON)) \
            | (P.F_MOVING if self.servo.moving() else 0) \
            | (P.F_TORQUE_ON if self.servo.torque else 0)
        if now >= self.next_telem:
            t_ms = int((now - self.t0) * 1000.0)
            out.append(self._frame(P.T_TELEM, P.pack_telem(
                t_ms, self.servo.measure(), self.state, self.flags, self.last_cmd_id or 0)))
            self.next_telem = max(self.next_telem + self.telem_period, now)
        if now >= self.next_diag:
            c = self.cnt
            out.append(self._frame(P.T_DIAG, P.pack_diag((
                c['rx_ok'], c['crc_err'], c['cobs_err'], c['len_err'], c['timeout'], c['overflow'],
                c['stale_cmd'], c['wd_trips'], min(65535, int(self.max_cmd_gap * 1000)), 0, 0))))
            self.next_diag = now + 1.0
        return out


# ====================================================================== транспорт
class Transport(object):
    def read(self):
        raise NotImplementedError

    def write(self, data):
        raise NotImplementedError


class TcpTransport(Transport):
    def __init__(self, host, port):
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind((host, port))
        self.srv.listen(1)
        self.srv.setblocking(False)
        self.conn = None
        self.name = 'socket://%s:%d' % (host, port)

    def read(self):
        if self.conn is None:
            try:
                self.conn, _ = self.srv.accept()
                self.conn.setblocking(False)
                self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except (BlockingIOError, OSError):
                return b''
        try:
            data = self.conn.recv(4096)
            if not data:                             # клиент отключился
                self.conn.close()
                self.conn = None
            return data
        except (BlockingIOError, InterruptedError):
            return b''
        except OSError:
            self.conn = None
            return b''

    def write(self, data):
        if self.conn is None:
            return
        try:
            self.conn.sendall(data)
        except OSError:
            self.conn = None


class PtyTransport(Transport):
    def __init__(self):
        import tty
        self.master, slave = os.openpty()
        tty.setraw(slave)
        self.name = os.ttyname(slave)
        self._slave = slave                           # держим открытым, иначе EIO на master
        os.set_blocking(self.master, False)

    def read(self):
        try:
            return os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            return b''

    def write(self, data):
        try:
            os.write(self.master, data)
        except OSError:
            pass


def parse_stuck(items):
    out = {}
    for it in items or []:
        j, t = it.split('@')
        out[int(j) - 1] = float(t)
    return out


def run(transport, sim, duration=None, drop=0.0, ber=0.0, rng=None):
    rng = rng or random.Random(3)
    t_prev = time.monotonic()
    t_end = None if duration is None else t_prev + duration
    while t_end is None or time.monotonic() < t_end:
        now = time.monotonic()
        data = transport.read()
        out = sim.feed(data, now) if data else []
        out.extend(sim.tick(now, now - t_prev))
        t_prev = now
        for frame in out:
            if drop and rng.random() < drop:
                continue
            if ber:
                frame = bytes(b ^ (1 << rng.randrange(8)) if rng.random() < ber * 8 else b
                              for b in frame)
            transport.write(frame)
        time.sleep(0.0005)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Виртуальный стенд ПОУК: модель МК и приводов (протокол v2)')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5760)
    ap.add_argument('--pty', action='store_true', help='псевдотерминал вместо TCP (Linux)')
    ap.add_argument('--wd-ms', type=int, default=60, help='тайм-аут сторожевого таймера, мс')
    ap.add_argument('--telem-hz', type=float, default=100.0)
    ap.add_argument('--stuck', action='append', help='заклинивание: <сустав 1..6>@<секунды от старта>')
    ap.add_argument('--drop', type=float, default=0.0, help='вероятность потери кадра МК -> верх')
    ap.add_argument('--ber', type=float, default=0.0, help='вероятность искажения бита МК -> верх')
    ap.add_argument('--no-noise', action='store_true')
    ap.add_argument('--seed', type=int, default=7)
    ap.add_argument('--duration', type=float, default=None)
    a = ap.parse_args(argv)
    sim = SimMcu(wd_ms=a.wd_ms, telem_hz=a.telem_hz, rng=random.Random(a.seed), noise=not a.no_noise)
    sim.servo.stuck = parse_stuck(a.stuck)
    tr = PtyTransport() if a.pty else TcpTransport(a.host, a.port)
    print('Виртуальный стенд: порт %s, T_wd = %d мс, телеметрия %.0f Гц' % (tr.name, a.wd_ms, a.telem_hz))
    sys.stdout.flush()
    try:
        run(tr, sim, a.duration, a.drop, a.ber, random.Random(a.seed + 1))
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())

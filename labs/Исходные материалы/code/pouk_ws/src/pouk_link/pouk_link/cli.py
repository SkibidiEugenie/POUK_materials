# -*- coding: utf-8 -*-
"""ЛР2: приложение одноплатника — управление стендом по протоколу v2.

Примеры (на NanoPi порт /dev/ttyS1; на ПК с виртуальным стендом socket://127.0.0.1:5760):
    python3 -m pouk_link.cli --port /dev/ttyS1 monitor --seconds 5
    python3 -m pouk_link.cli --port /dev/ttyS1 home
    python3 -m pouk_link.cli --port /dev/ttyS1 goto 120 0 30
    python3 -m pouk_link.cli --port /dev/ttyS1 --log run1 circle --laps 2
    python3 -m pouk_link.cli --port /dev/ttyS1 --log hour load --duration 3600
    python3 -m pouk_link.cli --port /dev/ttyS1 --log wd watchdog-test --stop-after 3
    python3 -m pouk_link.cli --port /dev/ttyS1 --ber 1e-4 --drop 0.01 stats --seconds 30

Журналы (--log PREFIX): PREFIX_telem.csv и PREFIX_cmd.csv; обработка — pouk_link.analyze.
"""
import argparse
import math
import signal
import sys
import time

from . import kinematics as K
from . import protocol as P
from .link import FaultInjector, StandLink

EPS = 0.08            # рад, допуск достижения (§А.5: порядка люфта)


def _print_stats(link):
    st = link.stats()
    keys = ['telem_ok', 'telem_lost', 'rx_packets', 'crc_err', 'cobs_err', 'len_err', 'type_err',
            'timeout', 'overflow', 'cmd_sent', 'cmd_acked', 'cmd_rejected', 'cmd_retries', 'cmd_lost',
            'ack_unexpected', 'mcu_restarts', 'inj_drop', 'inj_corrupt', 'inj_trunc']
    print('Одноплатник: ' + ', '.join('%s=%d' % (k, st.get(k, 0)) for k in keys))
    d = link.latest_diag()
    if d is not None:
        print('Микроконтроллер: ' + ', '.join('%s=%d' % kv for kv in d._asdict().items()
                                              if kv[0] != 'reserved'))
    rtt = sorted(link.tracker.rtt)
    if rtt:
        def pct(p):
            return rtt[min(len(rtt) - 1, int(round(p / 100.0 * (len(rtt) - 1))))] * 1000
        print('Оборот команда-подтверждение, мс: p50=%.2f p99=%.2f max=%.2f (n=%d)'
              % (pct(50), pct(99), rtt[-1] * 1000, len(rtt)))


def _state_line(s):
    return '%-7s q=[%s] flags=0x%02X' % (P.STATE_NAMES.get(s.state, '?'),
                                         ' '.join('%+.3f' % a for a in s.q), s.flags)


def _start_run(link, q_start=None):
    """Перевести МК в RUN и сразу начать поток уставок (иначе сработает сторожевой таймер)."""
    s = link.wait_telemetry()
    if s is None:
        raise RuntimeError('нет телеметрии: проверьте порт, питание и прошивку')
    link.set_target(q_start if q_start is not None else s.q)
    link.stream_start()
    r = link.set_mode('run')
    if not r.ok:
        raise RuntimeError('МК не перешёл в RUN: %s' % (r,))
    return s


def cmd_monitor(link, a):
    """Режим и углы стенда; --seconds 0 — до остановки (служба systemd, шаг 10 ЛР2)."""
    forever = a.seconds <= 0
    t_end = time.monotonic() + a.seconds
    period = 1.0 if forever else 0.2
    n = 0
    while forever or time.monotonic() < t_end:
        s = link.latest()
        print(_state_line(s) if s is not None else 'нет телеметрии', flush=True)
        n += 1
        if forever and n % 60 == 0:
            _print_stats(link)                  # раз в минуту — счётчики канала в журнал службы
            sys.stdout.flush()
        time.sleep(period)
    _print_stats(link)


def cmd_mode(link, a):
    link.wait_telemetry()
    print(link.set_mode(a.mode))


def cmd_home(link, a):
    _start_run(link)
    home = [0.0] * P.N_JOINTS
    link.set_target(home, a.vmax)
    ok = link.wait_reached(home, EPS, a.timeout)
    print('HOME:', 'достигнуто' if ok else 'тайм-аут')
    return 0 if ok else 2


def cmd_goto(link, a):
    r = K.ik(a.x / 1000.0, a.y / 1000.0, a.z / 1000.0)
    if not r.ok:
        print('ОЗК: отказ —', r.reason)
        return 2
    _start_run(link)
    q = K.with_gripper(r.q)
    link.set_target(q, a.vmax)
    ok = link.wait_reached(q, EPS, a.timeout)
    s = link.latest()
    x, y, z = K.fk(s.q)
    print('GOTO: %s; факт (%.1f, %.1f, %.1f) мм' % ('достигнуто' if ok else 'тайм-аут',
                                                    x * 1e3, y * 1e3, z * 1e3))
    return 0 if ok else 2


def circle_setpoints(a):
    pts = K.circle_points(a.cx / 1e3, a.cy / 1e3, a.cz / 1e3, a.r / 1e3, a.n)
    qs = []
    for i, (x, y, z) in enumerate(pts):
        r = K.ik(x, y, z)
        if not r.ok:
            raise ValueError('точка %d недостижима: %s' % (i, r.reason))
        qs.append(K.with_gripper(r.q))
    return qs


def cmd_circle(link, a):
    qs = circle_setpoints(a)
    print('Окружность: R=%.1f мм, N=%d, стрелка сегмента %.3f мм' % (a.r, a.n, K.sagitta(a.r, a.n)))
    _start_run(link)
    link.set_target(qs[0], a.vmax)
    if not link.wait_reached(qs[0], EPS, a.timeout):
        print('CIRCLE: не удалось выйти в начальную точку')
        return 2
    if a.timed:
        # уставки как функция времени: q(t) по окружности с периодом a.timed секунд
        t0 = time.monotonic()
        while time.monotonic() - t0 < a.timed * a.laps:
            th = 2 * math.pi * ((time.monotonic() - t0) / a.timed)
            r = K.ik((a.cx + a.r * math.cos(th)) / 1e3, (a.cy + a.r * math.sin(th)) / 1e3, a.cz / 1e3)
            if r.ok:
                link.set_target(K.with_gripper(r.q), a.vmax)
            time.sleep(1.0 / link.stream_hz)
    else:
        skipped = 0
        for lap in range(a.laps):
            for i, q in enumerate(qs):
                link.set_target(q, a.vmax)
                if not link.wait_reached(q, EPS, a.timeout):
                    skipped += 1
                    print('CIRCLE: круг %d, точка %d — тайм-аут' % (lap + 1, i))
        print('CIRCLE: готово, пропущено точек: %d' % skipped)
    return 0


def cmd_load(link, a):
    """Длительный прогон под нагрузкой: окружности по времени, журнал для гистограмм."""
    qs = circle_setpoints(a)
    _start_run(link, qs[0])
    t_end = time.monotonic() + a.duration
    i = 0
    last_print = 0.0
    while time.monotonic() < t_end:
        link.set_target(qs[i % len(qs)], a.vmax)
        i += 1
        time.sleep(a.dwell)
        if time.monotonic() - last_print > 60:
            last_print = time.monotonic()
            _print_stats(link)
    _print_stats(link)
    return 0


def cmd_watchdog_test(link, a):
    """Движение, затем прекращение потока команд: МК обязан перейти в WD_STOP."""
    s = _start_run(link)
    q = list(s.q)
    q[0] = max(K.JOINT_MIN[0], min(K.JOINT_MAX[0], q[0] + a.swing))
    link.set_target(q, a.vmax)
    time.sleep(a.stop_after)
    t_stop = time.monotonic()
    link.stream_stop()
    print('Поток команд остановлен в t=%.3f' % t_stop)
    while time.monotonic() - t_stop < 2.0:
        s = link.latest()
        if s is not None and s.state == P.ST_WD_STOP:
            print('WD_STOP через %.0f мс после последней команды' % ((s.t_host - t_stop) * 1000))
            break
        time.sleep(0.002)
    else:
        print('!! МК не перешёл в WD_STOP за 2 с — сторожевой таймер не работает')
        return 2
    time.sleep(1.0)
    return 0


def cmd_stats(link, a):
    s = _start_run(link)
    link.set_target(s.q)
    time.sleep(a.seconds)
    _print_stats(link)
    return 0


def build_parser():
    ap = argparse.ArgumentParser(prog='pouk_link.cli', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyS1')
    ap.add_argument('--baud', type=int, default=P.BAUD)
    ap.add_argument('--stream-hz', type=float, default=50.0, help='частота выдачи уставок')
    ap.add_argument('--ack-timeout-ms', type=float, default=8.0)
    ap.add_argument('--log', metavar='PREFIX', help='журналы PREFIX_telem.csv, PREFIX_cmd.csv')
    ap.add_argument('--drop', type=float, default=0.0, help='помеха: вероятность потери кадра')
    ap.add_argument('--ber', type=float, default=0.0, help='помеха: вероятность инверсии бита')
    ap.add_argument('--trunc', type=float, default=0.0, help='помеха: вероятность обрыва кадра')
    ap.add_argument('--vmax', type=float, default=0.8, help='скорость приводов, рад/с')
    ap.add_argument('--timeout', type=float, default=6.0, help='тайм-аут достижения точки, с')
    sub = ap.add_subparsers(dest='cmd')
    p = sub.add_parser('monitor'); p.add_argument('--seconds', type=float, default=5.0, help='0 — до остановки')
    p = sub.add_parser('mode'); p.add_argument('mode', choices=['run', 'hold', 'relax'])
    sub.add_parser('home')
    p = sub.add_parser('goto')
    for c in 'xyz':
        p.add_argument(c, type=float, help='мм')
    for name in ('circle', 'load'):
        p = sub.add_parser(name)
        p.add_argument('--r', type=float, default=30.0)
        p.add_argument('--cx', type=float, default=100.0)
        p.add_argument('--cy', type=float, default=0.0)
        p.add_argument('--cz', type=float, default=30.0)
        p.add_argument('--n', type=int, default=20)
        if name == 'circle':
            p.add_argument('--laps', type=int, default=2)
            p.add_argument('--timed', type=float, default=0.0, metavar='PERIOD_S',
                           help='уставки по времени с периодом круга PERIOD_S (вместо обхода точек)')
        else:
            p.add_argument('--duration', type=float, default=3600.0)
            p.add_argument('--dwell', type=float, default=0.25, help='пауза между точками, с')
    p = sub.add_parser('watchdog-test')
    p.add_argument('--stop-after', type=float, default=1.0, help='через сколько секунд прекратить поток')
    p.add_argument('--swing', type=float, default=0.6, help='размах движения основания, рад')
    p = sub.add_parser('stats'); p.add_argument('--seconds', type=float, default=10.0)
    return ap


def _terminate(signum, frame):
    raise KeyboardInterrupt


COMMANDS = {'monitor': cmd_monitor, 'mode': cmd_mode, 'home': cmd_home, 'goto': cmd_goto,
            'circle': cmd_circle, 'load': cmd_load, 'watchdog-test': cmd_watchdog_test,
            'stats': cmd_stats}


def main(argv=None):
    ap = build_parser()
    a = ap.parse_args(argv)
    # systemd останавливает службу сигналом SIGTERM: завершаемся так же, как по Ctrl+C,
    # чтобы закрыть порт и дописать журналы
    signal.signal(signal.SIGTERM, _terminate)
    if not a.cmd:
        ap.print_help()
        return 1
    link = StandLink(a.port, a.baud, ack_timeout=a.ack_timeout_ms / 1000.0,
                     fault=FaultInjector(a.drop, a.ber, a.trunc))
    link.stream_hz = a.stream_hz
    try:
        link.open()
    except Exception as e:
        print('Не удалось открыть порт %s: %s' % (a.port, e))
        return 1
    if a.log:
        link.log_to(a.log)
    link.start()
    rc = 1
    try:
        rc = COMMANDS[a.cmd](link, a) or 0
    except KeyboardInterrupt:
        rc = 130
    except (RuntimeError, ValueError) as e:
        print('Ошибка:', e)
    finally:
        link.stream_stop()
        if link.latest() is not None and a.cmd not in ('mode', 'watchdog-test', 'monitor'):
            link.set_mode('hold')              # определённое состояние при выходе
        link.close()
    return rc


if __name__ == '__main__':
    sys.exit(main())

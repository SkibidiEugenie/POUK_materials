# -*- coding: utf-8 -*-
"""Зонд часов между машинами по схеме NTP (ЛР3 ★, ЛР4-Б, чек-лист bringup п. 1).

    ros2 run pouk_bringup clock_probe server                 # на одной машине (NanoPi, NUC)
    ros2 run pouk_bringup clock_probe client --n 50          # на другой (ПК)
Клиент печатает смещение часов θ и задержку δ. Лучшая оценка смещения — по обмену
с минимальной задержкой: у него наименьшая асимметрия пути (идея фильтра часов NTP).
Если имя сервера другое (несколько пар машин), задайте --name.
"""
import argparse
import time

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time

from pouk_interfaces.srv import ClockProbe

from .checklib import spin_in_background, wait_future


def to_msg(ns):
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


def to_ns(t):
    return t.sec * 1000000000 + t.nanosec


def serve(name):
    node = Node('clock_probe_server')

    def handle(req, res):
        t2 = time.time_ns()
        res.t1 = req.t1
        res.t2 = to_msg(t2)
        res.t3 = to_msg(time.time_ns())
        return res

    node.create_service(ClockProbe, name, handle)
    print('сервер зонда часов: %s' % node.resolve_service_name(name))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def probe(name, n, period, tolerance_ms):
    node = Node('clock_probe_client')
    ex = spin_in_background(node)
    cli = node.create_client(ClockProbe, name)
    if not cli.wait_for_service(timeout_sec=10.0):
        print('нет сервера %s: запущен ли clock_probe server и совпадает ли ROS_DOMAIN_ID?' % name)
        return 2
    samples = []
    for _ in range(n):
        t4_box = []
        t1 = time.time_ns()
        fut = cli.call_async(ClockProbe.Request(t1=to_msg(t1)))
        # t4 фиксируется в момент прихода ответа (обработчик завершения в потоке исполнителя),
        # а не после опроса в этом потоке: иначе задержка опроса войдёт в θ как асимметрия
        fut.add_done_callback(lambda f: t4_box.append(time.time_ns()))
        res = wait_future(fut, 2.0)
        if res is None or not t4_box:
            continue
        t4 = t4_box[0]
        t2, t3 = to_ns(res.t2), to_ns(res.t3)
        theta = ((t2 - t1) + (t3 - t4)) / 2.0
        delta = (t4 - t1) - (t3 - t2)
        samples.append((delta, theta))
        time.sleep(period)
    ex.shutdown()
    if not samples:
        print('ни одного ответа')
        return 2
    samples.sort()
    d_min, theta_best = samples[0]
    thetas = sorted(s[1] for s in samples)
    deltas = [s[0] for s in samples]
    print('обменов: %d' % len(samples))
    print('задержка δ, мс: мин %.2f, медиана %.2f, макс %.2f' % (d_min / 1e6, deltas[len(deltas) // 2] / 1e6,
                                                                 deltas[-1] / 1e6))
    print('смещение θ по обмену с мин. задержкой: %+.2f мс (погрешность не более ±%.2f мс)'
          % (theta_best / 1e6, d_min / 2e6))
    print('разброс θ по всем обменам: %+.2f … %+.2f мс' % (thetas[0] / 1e6, thetas[-1] / 1e6))
    if tolerance_ms is not None:
        ok = abs(theta_best) / 1e6 + d_min / 2e6 <= tolerance_ms
        print('критерий |θ| + δ/2 ≤ %.2f мс: %s' % (tolerance_ms, 'ВЫПОЛНЕН' if ok else 'НЕ ВЫПОЛНЕН'))
        return 0 if ok else 1
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('role', choices=['server', 'client'])
    ap.add_argument('--name', default='clock_probe')
    ap.add_argument('--n', type=int, default=50)
    ap.add_argument('--period', type=float, default=0.05)
    ap.add_argument('--tolerance-ms', type=float, default=None, help='δt_max из расчёта §5.4.3')
    args, _ = ap.parse_known_args()
    rclpy.init()
    try:
        code = serve(args.name) if args.role == 'server' else probe(args.name, args.n, args.period,
                                                                     args.tolerance_ms)
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code or 0)


if __name__ == '__main__':
    main()

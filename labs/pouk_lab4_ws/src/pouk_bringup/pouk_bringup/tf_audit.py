# -*- coding: utf-8 -*-
"""Проверка дерева координат (ЛР4-Б, чек-лист bringup п. 3; пособие, §5.5).

    ros2 run pouk_bringup tf_audit --seconds 5 --rate map:odom=20 odom:base_link=50
Проверяется: один корень и связность; у каждого кадра один родитель; ребро не публикуется
одновременно как статическое и динамическое; нет циклов; частоты рёбер не ниже заявленных;
метки времени не из будущего (признак рассинхронизации часов издателя и наблюдателя).
"""
import argparse
import collections
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from tf2_msgs.msg import TFMessage

from .checklib import Report, spin_in_background


class Collector(Node):
    def __init__(self):
        super().__init__('tf_audit')
        self.parents = collections.defaultdict(set)       # child -> {parent}
        self.kind = collections.defaultdict(set)          # (parent, child) -> {'static', 'dynamic'}
        self.stamps = collections.defaultdict(list)       # (parent, child) -> [(t_recv, t_stamp)]
        static_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf', lambda m: self._on(m, 'dynamic'), 100)
        self.create_subscription(TFMessage, '/tf_static', lambda m: self._on(m, 'static'), static_qos)

    def _on(self, msg, kind):
        now = time.time()
        for t in msg.transforms:
            edge = (t.header.frame_id.lstrip('/'), t.child_frame_id.lstrip('/'))
            self.parents[edge[1]].add(edge[0])
            self.kind[edge].add(kind)
            self.stamps[edge].append((now, t.header.stamp.sec + t.header.stamp.nanosec * 1e-9))


def audit(c, seconds, rates, future_ms):
    rep = Report('Дерево координат')
    edges = sorted(c.kind)
    frames = set(c.parents) | {p for ps in c.parents.values() for p in ps}
    rep.check('преобразования получены', bool(edges), '%d рёбер, %d кадров' % (len(edges), len(frames)))
    if not edges:
        return rep.summary()
    multi = {ch: sorted(ps) for ch, ps in c.parents.items() if len(ps) > 1}
    rep.check('у каждого кадра один родитель', not multi, '; '.join('%s <- %s' % kv for kv in multi.items()))
    both = [e for e in edges if len(c.kind[e]) > 1]
    rep.check('нет рёбер одновременно статических и динамических', not both, str(both))
    roots = sorted(f for f in frames if f not in c.parents)
    rep.check('один корень', len(roots) == 1, 'корни: %s' % roots)
    cyclic = []
    for f in frames:
        seen, cur = set(), f
        while cur in c.parents and cur not in seen:
            seen.add(cur)
            cur = sorted(c.parents[cur])[0]
        if cur in seen:
            cyclic.append(f)
    rep.check('нет циклов', not cyclic, str(sorted(cyclic)))
    print('\nребро                                   вид       частота, Гц   макс. «из будущего», мс')
    worst_future = 0.0
    for e in edges:
        st = c.stamps[e]
        dyn = 'dynamic' in c.kind[e]
        hz = (len(st) - 1) / (st[-1][0] - st[0][0]) if dyn and len(st) > 2 and st[-1][0] > st[0][0] else 0.0
        fut = max((ts - tr) * 1000.0 for tr, ts in st) if dyn else 0.0
        worst_future = max(worst_future, fut)
        print('%-40s %-9s %8.1f      %8.1f' % ('%s -> %s' % e, 'dyn' if dyn else 'static', hz, fut))
        want = rates.get(e)
        if want is not None:
            rep.check('частота %s -> %s не ниже %.0f Гц' % (e[0], e[1], want), hz >= 0.9 * want, '%.1f Гц' % hz)
    rep.check('метки не из будущего более чем на %.0f мс' % future_ms, worst_future <= future_ms,
              'худшее %.1f мс%s' % (worst_future, '' if worst_future <= future_ms else
                                    ': часы издателя впереди часов наблюдателя'))
    return rep.summary()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=5.0)
    ap.add_argument('--rate', nargs='*', default=[], help='parent:child=Гц — заявленные частоты')
    ap.add_argument('--future-ms', type=float, default=5.0)
    args, _ = ap.parse_known_args()
    rates = {}
    for item in args.rate:
        edge, hz = item.split('=')
        parent, child = edge.split(':')
        rates[(parent, child)] = float(hz)
    rclpy.init()
    node = Collector()
    ex = spin_in_background(node)
    time.sleep(args.seconds)
    failures = audit(node, args.seconds, rates, args.future_ms)
    ex.shutdown()
    rclpy.shutdown()
    raise SystemExit(1 if failures else 0)


if __name__ == '__main__':
    main()

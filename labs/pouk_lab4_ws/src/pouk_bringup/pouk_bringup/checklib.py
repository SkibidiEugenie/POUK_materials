# -*- coding: utf-8 -*-
"""Общие средства программ самопроверки: узел в фоне, ожидания, таблица результатов."""
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
REGISTER = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


class Probe(Node):
    """Наблюдатель: последние joint_states (с частотой и метками), режим стенда, произвольные строки."""

    def __init__(self, name):
        super().__init__(name)
        self.lock = threading.Lock()
        self.js = None
        self.js_times = []
        self.js_stamps = []
        self.strings = {}
        self.create_subscription(JointState, 'joint_states', self._on_js, REGISTER)
        self.watch_string('stand/state')

    def watch_string(self, topic):
        self.strings.setdefault(topic, None)
        self.create_subscription(String, topic, lambda m, t=topic: self._on_string(t, m), LATCHED)

    def _on_js(self, msg):
        with self.lock:
            self.js = dict(zip(msg.name, msg.position))
            self.js_times.append(time.monotonic())
            self.js_stamps.append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            del self.js_times[:-500]
            del self.js_stamps[:-500]

    def _on_string(self, topic, msg):
        with self.lock:
            self.strings[topic] = msg.data

    def q(self, names):
        with self.lock:
            return None if self.js is None else [self.js.get(n) for n in names]

    def string(self, topic):
        with self.lock:
            return self.strings.get(topic)


def spin_in_background(node):
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    return ex


def wait_for(pred, timeout, period=0.02):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        v = pred()
        if v:
            return v
        time.sleep(period)
    return pred()


def wait_future(fut, timeout):
    t_end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < t_end:
        time.sleep(0.01)
    return fut.result() if fut.done() else None


def call(node, client, request, timeout=5.0):
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    return wait_future(client.call_async(request), timeout)


class Report(object):
    def __init__(self, title):
        self.title = title
        self.rows = []

    def check(self, name, ok, detail=''):
        self.rows.append((name, bool(ok), detail))
        print('%s  %-58s %s' % ('PASS' if ok else 'FAIL', name, detail), flush=True)
        return ok

    def summary(self):
        failed = [r for r in self.rows if not r[1]]
        print('\n%s: %d проверок, %d не пройдено' % (self.title, len(self.rows), len(failed)))
        for name, _, detail in failed:
            print('  - %s: %s' % (name, detail))
        return len(failed)


def run_main(check_fn, node_name):
    rclpy.init()
    probe = Probe(node_name)
    ex = spin_in_background(probe)
    try:
        failures = check_fn(probe)
    finally:
        ex.shutdown()
        probe.destroy_node()
        rclpy.try_shutdown() if hasattr(rclpy, 'try_shutdown') else rclpy.shutdown()
    raise SystemExit(1 if failures else 0)

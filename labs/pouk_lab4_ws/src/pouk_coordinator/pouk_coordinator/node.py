# -*- coding: utf-8 -*-
"""Узел-координатор (ЛР3): автомат fsm.Coordinator за интерфейсами ROS 2.

  ~/home, ~/stop, ~/reset   std_srvs/Trigger
  ~/move_to                 pouk_interfaces/MoveTo
  ~/circle                  pouk_interfaces/Circle
  ~/state                   std_msgs/String   имя состояния (опоздавший подписчик получает текущее)
  ~/status                  std_msgs/String   пояснение: цель, номер точки, причина отказа
  stand/joint_command       sensor_msgs/JointState   уставка мосту — только при её изменении
Вход: joint_states (факт) и stand/state (режим стенда от моста).
"""
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pouk_interfaces.srv import Circle, MoveTo

from .fsm import Coordinator

DEFAULT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint']


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator')
        p = self.declare_parameter
        self.names = list(p('joint_names', DEFAULT_NAMES).value)
        self.core = Coordinator(
            reach_eps=p('reach_eps', 0.08).value,
            reach_timeout=p('reach_timeout', 8.0).value,
            vmax=p('vmax', 0.8).value,
            home=list(p('home', [0.0] * 6).value),
            circle_defaults=(p('circle.cx', 0.100).value, p('circle.cy', 0.0).value,
                             p('circle.cz', 0.030).value, p('circle.r', 0.030).value,
                             p('circle.n', 20).value, p('circle.laps', 1).value))
        self.stale_s = p('joint_states_stale_s', 0.5).value
        rate = p('rate', 20.0).value

        qos_register = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                  reliability=ReliabilityPolicy.BEST_EFFORT,
                                  durability=DurabilityPolicy.VOLATILE)
        qos_latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_cmd = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        self.pub_cmd = self.create_publisher(JointState, 'stand/joint_command', qos_cmd)
        self.pub_state = self.create_publisher(String, '~/state', qos_latched)
        self.pub_status = self.create_publisher(String, '~/status', qos_latched)
        self.create_subscription(JointState, 'joint_states', self._on_joint_states, qos_register)
        self.create_subscription(String, 'stand/state', self._on_stand_state, qos_latched)
        self.create_service(Trigger, '~/home', self._srv_home)
        self.create_service(Trigger, '~/stop', self._srv_stop)
        self.create_service(Trigger, '~/reset', self._srv_reset)
        self.create_service(MoveTo, '~/move_to', self._srv_move_to)
        self.create_service(Circle, '~/circle', self._srv_circle)
        self.create_timer(1.0 / rate, self._tick)

        self._lock = threading.Lock()      # одно ядро: обработчики не должны пересекаться
        self._q = None
        self._q_t = 0.0
        self._stand = 'NO_LINK'
        self._sent_target = None
        self._published = (None, None)
        self._publish()

    # ---------------------------------------------------------------- входы
    def _on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in self.names):
            self._q = [pos[n] for n in self.names]
            self._q_t = time.monotonic()

    def _on_stand_state(self, msg):
        self._stand = msg.data

    def _stand_ok(self):
        fresh = self._q is not None and time.monotonic() - self._q_t < self.stale_s
        return fresh and self._stand == 'RUN', ('нет joint_states' if not fresh else 'стенд в режиме %s' % self._stand)

    # ---------------------------------------------------------------- выходы
    def _publish(self):
        c = self.core
        if (c.state, c.message) != self._published:
            if c.state != self._published[0]:
                self.pub_state.publish(String(data=c.state))
                self.get_logger().info('состояние: %s — %s' % (c.state, c.message))
            self.pub_status.publish(String(data=c.message))
            self._published = (c.state, c.message)
        if c.target is not None and c.target != self._sent_target:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.names)
            msg.position = [float(v) for v in c.target]
            self.pub_cmd.publish(msg)
            self._sent_target = list(c.target)

    def _tick(self):
        with self._lock:
            ok, reason = self._stand_ok()
            self.core.tick(time.monotonic(), self._q, ok, reason)
            self._publish()

    # ---------------------------------------------------------------- сервисы
    def _call(self, fn, res):
        with self._lock:
            res_ok, message = fn()
            self._publish()
        if hasattr(res, 'success'):
            res.success, res.message = res_ok, message
        else:
            res.accepted, res.message = res_ok, message
        return res

    def _srv_home(self, req, res):
        return self._call(lambda: self.core.request_home(time.monotonic(), self._stand_ok()[0]), res)

    def _srv_stop(self, req, res):
        return self._call(lambda: self.core.request_stop(time.monotonic(), self._q), res)

    def _srv_reset(self, req, res):
        return self._call(lambda: self.core.request_reset(time.monotonic(), self._stand_ok()[0]), res)

    def _srv_move_to(self, req, res):
        return self._call(lambda: self.core.request_move_to(req.x, req.y, req.z, req.vmax, time.monotonic(),
                                                            self._stand_ok()[0]), res)

    def _srv_circle(self, req, res):
        return self._call(lambda: self.core.request_circle(req.cx, req.cy, req.cz, req.r, req.n, req.laps,
                                                           time.monotonic(), self._stand_ok()[0]), res)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CoordinatorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-
"""Узел-мост стенда: ROS 2 <-> протокол стенда v2 (ЛР3, ЛР4-А).

Мост отвечает ТОЛЬКО за коммуникацию (пособие, §3.9, п. 3): протокол и канал —
библиотека pouk_link, интерполяция и допуски — ядро executor без ROS, логика
сценариев — координатор. Здесь — отображение на топики, сервисы и действие.

Интерфейсы (имена относительные: узел можно запустить в пространстве имён):
  joint_states                     sensor_msgs/JointState       все 6 суставов, метка — момент измерения
  stand/joint_command              sensor_msgs/JointState       уставка по именам (можно часть суставов)
  stand/state                      std_msgs/String              BOOT/HOLD/RUN/WD_STOP/RELAX/FAULT/NO_LINK
  diagnostics                      diagnostic_msgs/DiagnosticArray  счётчики канала и МК, 1 Гц
  stand/run, stand/hold, stand/relax   std_srvs/Trigger         смена режима МК
  arm_controller/follow_joint_trajectory  control_msgs/FollowJointTrajectory  исполнитель траектории

Параметры — см. declare_parameter ниже и config/*.yaml пакета pouk_bringup.
"""
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pouk_link import kinematics as K
from pouk_link import protocol as P
from pouk_link.link import StandLink

from . import executor as E
from .joint_map import JointMap, JointMapError

DEFAULT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'gripper_joint']


def _dur(d):
    return d.sec + d.nanosec * 1e-9


class StandBridge(Node):
    def __init__(self):
        super().__init__('stand_bridge')
        p = self.declare_parameter
        port = p('port', '/dev/ttyS1').value
        baud = p('baud', 115200).value
        names = list(p('joint_names', DEFAULT_NAMES).value)
        signs = list(p('joint_signs', [1.0] * 6).value)
        offsets = list(p('joint_offsets', [0.0] * 6).value)
        self.stream_hz = p('stream_hz', 50.0).value
        self.js_rate = p('joint_states_rate', 50.0).value
        self.command_vmax = p('command_vmax', 1.2).value
        self.traj_vmax = p('trajectory_vmax', 2.0).value
        self.auto_run = p('auto_run', False).value
        self.stale_s = p('telemetry_stale_s', 0.25).value
        self.exec_rate = p('exec_rate', 100.0).value
        self.max_stamp_age = p('max_stamp_age', 0.5).value
        connect_timeout = p('connect_timeout', 10.0).value
        log_prefix = p('log_prefix', '').value
        action_name = p('action_name', 'arm_controller/follow_joint_trajectory').value
        self.exe = E.TrajectoryExecutor(
            path_tolerance=p('path_tolerance', 0.15).value,
            goal_tolerance=p('goal_tolerance', 0.05).value,
            goal_time_tolerance=p('goal_time_tolerance', 1.0).value,
            start_tolerance=p('start_tolerance', 0.10).value)

        self.map = JointMap(names, signs, offsets)
        # пределы в именах и знаках ROS: q_ROS = (q_стенд − offset) · sign
        self.limits_ros = {}
        for i, n in enumerate(names):
            a = (K.JOINT_MIN[i] - offsets[i]) * signs[i]
            b = (K.JOINT_MAX[i] - offsets[i]) * signs[i]
            self.limits_ros[n] = (min(a, b), max(a, b))

        # >>> STUDENT ЛР3-шаг3: профили QoS. Подберите и обоснуйте в отчёте (пособие, §3.4):
        # >>> состояние суставов (издатель должен быть совместим и с robot_state_publisher,
        # >>> и с «регистровыми» читателями), командный топик (важна последняя уставка),
        # >>> режим стенда (опоздавший подписчик должен сразу узнать текущий режим).
        raise NotImplementedError('ЛР3, шаг 3: профили QoS моста')
        # <<< STUDENT
        self.pub_js = self.create_publisher(JointState, 'joint_states', qos_state)
        self.pub_state = self.create_publisher(String, 'stand/state', qos_mode)
        self.pub_diag = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        self._lock = threading.Lock()
        self._target = None             # последняя отправленная уставка, вектор стенда
        self._active_goal = None
        self._exec_lock = threading.Lock()
        self._preempt = threading.Event()
        self._published_state = None
        self._decim = max(1, int(round(100.0 / max(1.0, self.js_rate))))
        self._n_telem = 0
        self._errors_prev = 0
        self._run_acked_t = 0.0
        self.counters = {'cmd_topic_ok': 0, 'cmd_topic_rejected': 0, 'goals': 0,
                         'goals_succeeded': 0, 'goals_aborted': 0}

        # канал к МК: порт открывается с повторами — виртуальный стенд может стартовать позже моста
        self.link = StandLink(port, baud)
        t_end = time.monotonic() + connect_timeout
        while True:
            try:
                self.link.open()
                break
            except Exception as e:                            # noqa: B902 — нужен текст любой ошибки порта
                if time.monotonic() > t_end:
                    raise RuntimeError('не открыть порт %s: %s' % (port, e))
                time.sleep(0.5)
        if log_prefix:
            self.link.log_to(log_prefix)
        self.link.on_telemetry(self._on_sample)
        self.link.start()
        s = self.link.wait_telemetry(3.0)
        if s is None:
            self.get_logger().error('нет телеметрии от МК на %s: питание, прошивка, скорость порта?' % port)
        else:
            self.get_logger().info('стенд на связи: %s, сессия МК %d' % (P.STATE_NAMES[s.state], s.session))

        cb_srv = MutuallyExclusiveCallbackGroup()
        self.create_subscription(JointState, 'stand/joint_command', self._on_command, qos_cmd,
                                 callback_group=cb_srv)
        self.create_service(Trigger, 'stand/run', self._srv_run, callback_group=cb_srv)
        self.create_service(Trigger, 'stand/hold', self._srv_hold, callback_group=cb_srv)
        self.create_service(Trigger, 'stand/relax', self._srv_relax, callback_group=cb_srv)
        self.create_timer(1.0, self._publish_diagnostics, callback_group=cb_srv)
        self.create_timer(0.1, self._watch_link, callback_group=cb_srv)
        self._action = ActionServer(
            self, FollowJointTrajectory, action_name,
            execute_callback=self._execute, goal_callback=lambda goal: GoalResponse.ACCEPT,
            handle_accepted_callback=self._on_accepted,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        if self.auto_run and s is not None:
            ok, msg = self._enter_run()
            self.get_logger().info('автоматический переход в RUN: %s' % msg)

    # ================================================================ телеметрия
    def _on_sample(self, s):
        """Поток приёма pouk_link: каждый образец телеметрии (100 Гц)."""
        self._n_telem += 1
        if self._n_telem % self._decim == 0:
            msg = JointState()
            # метка — момент ИЗМЕРЕНИЯ в шкале часов ROS (s.stamp — шкала monotonic)
            stamp = s.stamp + (time.time() - time.monotonic())
            msg.header.stamp = Time(nanoseconds=int(stamp * 1e9)).to_msg()
            msg.name, msg.position = self.map.to_ros(s.q)
            self.pub_js.publish(msg)
        self._publish_state(P.STATE_NAMES.get(s.state, 'UNKNOWN'))

    def _publish_state(self, name):
        if name != self._published_state:
            self._published_state = name
            self.pub_state.publish(String(data=name))
            self.get_logger().info('режим стенда: %s' % name)

    def _fresh_sample(self):
        s = self.link.latest()
        if s is None or time.monotonic() - s.t_host > self.stale_s:
            return None
        return s

    def _watch_link(self):
        s = self._fresh_sample()
        if s is None:
            self._publish_state('NO_LINK')
            return
        # МК вышел из RUN сам (сторожевой таймер, перезапуск): поток уставок больше не нужен.
        # Судить можно только по образцу, принятому ПОЗЖЕ подтверждения RUN с запасом:
        # телеметрия, отправленная до обработки команды, ещё несёт старый режим (гонка!)
        if self.link.streaming and s.state != P.ST_RUN and s.t_host > self._run_acked_t + 0.05:
            self.link.stream_stop()

    # ================================================================ команды
    def _set_target(self, q, vmax):
        with self._lock:
            self._target = list(q)
        self.link.set_target(q, vmax)

    def _on_command(self, msg):
        # арбитраж: пока исполняется траектория, ресурс «руки» захвачен исполнителем
        if self._active_goal is not None:
            self.counters['cmd_topic_rejected'] += 1
            self.get_logger().warn('команда по топику отклонена: исполняется траектория',
                                   throttle_duration_sec=2.0)
            return
        s = self._fresh_sample()
        if s is None or s.state != P.ST_RUN:
            self.counters['cmd_topic_rejected'] += 1
            self.get_logger().warn('команда отклонена: стенд не в RUN (вызовите stand/run)',
                                   throttle_duration_sec=2.0)
            return
        try:
            with self._lock:
                base = self._target if self._target is not None else s.q
            q = self.map.to_stand(msg.name, msg.position, current=base)
        except JointMapError as e:
            self.counters['cmd_topic_rejected'] += 1
            self.get_logger().error('команда отклонена: %s' % e, throttle_duration_sec=2.0)
            return
        if not K.in_limits(q):
            self.counters['cmd_topic_rejected'] += 1
            self.get_logger().error('команда вне пределов суставов: %s' % ['%.3f' % v for v in q],
                                    throttle_duration_sec=2.0)
            return
        self.counters['cmd_topic_ok'] += 1
        self._set_target(q, self.command_vmax)

    # ================================================================ режимы
    def _enter_run(self):
        s = self._fresh_sample()
        if s is None:
            return False, 'нет телеметрии'
        if s.state == P.ST_RUN:
            return True, 'уже RUN'
        self._set_target(s.q, self.command_vmax)      # удерживать там, где стоим
        self.link.stream_start(self.stream_hz)        # поток — ДО режима: иначе сработает сторожевой таймер
        r = self.link.set_mode('run')
        if not r.ok:
            self.link.stream_stop()
            reason = P.REASON_NAMES.get(r.reason, 'нет подтверждения') if r.reason is not None else 'нет подтверждения'
            return False, 'МК отказал: %s' % reason
        self._run_acked_t = time.monotonic()
        return True, 'RUN'

    def _leave_run(self, mode):
        self._preempt.set()                            # прервать исполнение траектории
        r = self.link.set_mode(mode)
        self.link.stream_stop()
        return (True, mode.upper()) if r.ok else (False, 'МК не подтвердил %s' % mode.upper())

    def _srv_run(self, req, res):
        res.success, res.message = self._enter_run()
        return res

    def _srv_hold(self, req, res):
        res.success, res.message = self._leave_run('hold')
        return res

    def _srv_relax(self, req, res):
        res.success, res.message = self._leave_run('relax')
        return res

    # ================================================================ траектория
    def _on_accepted(self, goal_handle):
        if self._active_goal is not None:
            self._preempt.set()                        # новая цель вытесняет исполняемую
        goal_handle.execute()

    def _execute(self, goal_handle):
        with self._exec_lock:                          # дождаться завершения вытесненной цели
            self._preempt.clear()
            self._active_goal = goal_handle
            self.counters['goals'] += 1
            try:
                result = self._run_goal(goal_handle)
            finally:
                self._active_goal = None
        key = 'goals_succeeded' if result.error_code == E.SUCCESSFUL and not result.error_string else 'goals_aborted'
        self.counters[key] += 1
        return result

    def _hold_here(self, s):
        """Удержание текущего (измеренного) положения — безопасное состояние при отказе."""
        if s is not None:
            self._set_target(s.q, self.command_vmax)

    def _finish(self, goal_handle, code, message, s=None, hold=True, cancel=False):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        if code == E.SUCCESSFUL and not message:
            goal_handle.succeed()
        else:
            # >>> STUDENT ЛР4А-шаг5: семантика нарушения контракта. При любом отказе — сначала
            # >>> удержание измеренного положения (кроме вытеснения: новая цель продолжит
            # >>> движение), затем статус цели: canceled() для отмены клиентом, abort() для остального.
            goal_handle.abort()  # TODO: удержание и статус цели
            # <<< STUDENT
            self.get_logger().warn('траектория завершена с отказом %d: %s' % (code, message))
        return result

    def _run_goal(self, gh):
        goal = gh.request
        traj = goal.trajectory
        s = self._fresh_sample()
        if s is None:
            return self._finish(gh, E.INVALID_GOAL, 'нет телеметрии стенда', hold=False)
        if s.state != P.ST_RUN:
            return self._finish(gh, E.INVALID_GOAL, 'стенд в режиме %s, нужен RUN (сервис stand/run)'
                                % P.STATE_NAMES.get(s.state), hold=False)
        stamp = Time.from_msg(traj.header.stamp).nanoseconds
        if stamp != 0:
            delay = (stamp - self.get_clock().now().nanoseconds) * 1e-9
            if delay < -self.max_stamp_age:
                return self._finish(gh, E.OLD_HEADER_TIMESTAMP,
                                    'метка времени траектории устарела на %.2f с' % -delay, hold=False)
            if delay > 0:
                time.sleep(min(delay, 5.0))            # траектория назначена на будущий момент
                s = self._fresh_sample() or s
        names = list(traj.joint_names)
        actual = dict(zip(*self.map.to_ros(s.q)))
        fail = self.exe.start(
            names, [_dur(pt.time_from_start) for pt in traj.points],
            [list(pt.positions) for pt in traj.points], [list(pt.velocities) for pt in traj.points],
            [list(pt.accelerations) for pt in traj.points], actual, time.monotonic(),
            path_tol=[(t.name, t.position) for t in goal.path_tolerance],
            goal_tol=[(t.name, t.position) for t in goal.goal_tolerance],
            goal_time_tol=_dur(goal.goal_time_tolerance),
            known_names=self.map.names, limits=self.limits_ros)
        if fail is not None:
            return self._finish(gh, fail.code, fail.message, hold=False)
        self.get_logger().info('траектория: %d точек, %.2f с, интерполяция %s'
                               % (len(traj.points), self.exe.traj.duration, self.exe.traj.kind))

        period = 1.0 / self.exec_rate
        fb_every = max(1, int(self.exec_rate / 20.0))
        fb = FollowJointTrajectory.Feedback()
        fb.joint_names = names
        t_next = time.monotonic()
        k = 0
        while True:
            if gh.is_cancel_requested:
                return self._finish(gh, E.SUCCESSFUL, 'отменено клиентом', cancel=True)
            if self._preempt.is_set():
                return self._finish(gh, E.SUCCESSFUL, 'вытеснено новой целью или сменой режима', hold=False)
            s = self._fresh_sample()
            if s is None:
                return self._finish(gh, E.PATH_TOLERANCE_VIOLATED, 'STAND: пропала телеметрия')
            if s.state != P.ST_RUN:
                return self._finish(gh, E.PATH_TOLERANCE_VIOLATED,
                                    'STAND: стенд вышел из RUN в %s' % P.STATE_NAMES.get(s.state), s)
            actual = dict(zip(*self.map.to_ros(s.q)))
            st = self.exe.step(time.monotonic(), actual)
            with self._lock:
                base = self._target if self._target is not None else s.q
            self._set_target(self.map.to_stand(names, st.desired, current=base), self.traj_vmax)
            if st.status == E.DONE:
                return self._finish(gh, st.code, st.message, s)
            k += 1
            if k % fb_every == 0:
                fb.header.stamp = self.get_clock().now().to_msg()
                fb.desired.positions, fb.desired.velocities = st.desired, st.desired_vel
                fb.actual.positions = [actual[n] for n in names]
                fb.error.positions = st.error
                gh.publish_feedback(fb)
            t_next += period
            delay = t_next - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                t_next = time.monotonic()              # отстали: не догонять пачкой

    # ================================================================ диагностика
    def _publish_diagnostics(self):
        st = self.link.stats()
        d = self.link.latest_diag()
        s = self._fresh_sample()
        status = DiagnosticStatus()
        status.name = 'pouk_bridge: stand link'
        status.hardware_id = str(self.link.port)
        # предупреждение — по ПРИРОСТУ ошибок за последний период, а не по накопленной сумме
        errors = sum(st[k] for k in ('crc_err', 'cobs_err', 'len_err', 'timeout', 'cmd_lost', 'telem_lost'))
        new_errors = errors - self._errors_prev
        self._errors_prev = errors
        if s is None:
            status.level, status.message = DiagnosticStatus.ERROR, 'нет телеметрии'
        elif new_errors > 0:
            status.level, status.message = DiagnosticStatus.WARN, '+%d ошибок канала за период' % new_errors
        else:
            status.level, status.message = DiagnosticStatus.OK, P.STATE_NAMES.get(s.state, '?')
        values = dict(st)
        values.update(self.counters)
        if d is not None:
            values.update({'mcu_' + k: v for k, v in d._asdict().items() if k != 'reserved'})
        rtt = sorted(list(self.link.tracker.rtt)[-2000:])      # последние 2000 оборотов
        if rtt:
            values['rtt_p50_ms'] = round(rtt[len(rtt) // 2] * 1000, 2)
            values['rtt_p99_ms'] = round(rtt[int(0.99 * (len(rtt) - 1))] * 1000, 2)
            values['rtt_max_ms'] = round(rtt[-1] * 1000, 2)
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in sorted(values.items())]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self.pub_diag.publish(arr)

    # ================================================================ завершение
    def shutdown(self):
        s = self.link.latest()
        try:
            if s is not None and s.state == P.ST_RUN:
                self.link.set_mode('hold')             # явное удержание, а не срабатывание сторожевого таймера
        finally:
            self.link.stream_stop()
            self.link.close()


def main(args=None):
    # Поток уставок к МК живёт в том же процессе, что и исполнитель ROS: в Python потоки делят
    # GIL, и поток передачи ждёт его до интервала переключения (по умолчанию 5 мс) после
    # каждого пробуждения. Меньший интервал — меньше худший разрыв потока ценой пропускной
    # способности; измеряется счётчиком max_cmd_gap_ms в диагностике МК (ЛР3, шаг 4).
    sys.setswitchinterval(0.001)
    rclpy.init(args=args)
    node = None
    try:
        node = StandBridge()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

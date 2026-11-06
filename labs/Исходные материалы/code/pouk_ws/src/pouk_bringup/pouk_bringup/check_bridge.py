# -*- coding: utf-8 -*-
"""Самопроверка моста по контракту (ЛР3, шаги 2–3; ЛР4-А, шаги 3–5).

Запуск на ПК при работающем виртуальном стенде:
    ros2 launch pouk_bringup sim_stand.launch.py              # терминал 1
    ros2 run pouk_bringup check_bridge --lab3                  # терминал 2: только проверки ЛР3
    ros2 run pouk_bringup check_bridge                         # всё, включая траектории ЛР4-А
    ros2 launch pouk_bringup sim_stand.launch.py stuck:=2@25   # проверка допуска по пути
    ros2 run pouk_bringup check_bridge --stuck
На реальном стенде проверки с движением выполняйте только при свободной рабочей зоне.
"""
import argparse
import math
import time

from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .checklib import Report, call, run_main, wait_for, wait_future

ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
ALL = ARM + ['gripper_joint']
FJT = FollowJointTrajectory.Result


def duration(t):
    return Duration(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)))


def make_goal(points, names=ARM, with_vel=False, path_tol=None):
    """points — список (t, [q...]); скорости — центральные разности (для кубической интерполяции)."""
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(names)
    for k, (t, q) in enumerate(points):
        p = JointTrajectoryPoint(positions=[float(v) for v in q], time_from_start=duration(t))
        if with_vel:
            if 0 < k < len(points) - 1:
                (t0, q0), (t1, q1) = points[k - 1], points[k + 1]
                p.velocities = [(b - a) / (t1 - t0) for a, b in zip(q0, q1)]
            else:
                p.velocities = [0.0] * len(q)
        goal.trajectory.points.append(p)
    if path_tol is not None:
        goal.path_tolerance = [JointTolerance(name=n, position=path_tol) for n in names]
    return goal


class Bridge(object):
    def __init__(self, probe):
        self.probe = probe
        self.cmd = probe.create_publisher(JointState, 'stand/joint_command', 1)
        self.action = ActionClient(probe, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
        self.run = probe.create_client(Trigger, 'stand/run')
        self.hold = probe.create_client(Trigger, 'stand/hold')

    def send(self, goal):
        fut = self.action.send_goal_async(goal)
        gh = wait_future(fut, 5.0)
        return gh

    def result(self, gh, timeout):
        if gh is None or not gh.accepted:
            return None, None
        res = wait_future(gh.get_result_async(), timeout)
        return (res.status, res.result) if res is not None else (None, None)

    def command(self, names, values):
        self.cmd.publish(JointState(name=list(names), position=[float(v) for v in values]))

    def arm(self):
        return self.probe.q(ARM)


def checks(probe, args):
    rep = Report('Мост стенда')
    b = Bridge(probe)
    state = wait_for(lambda: probe.string('stand/state'), 10.0)
    if state != 'RUN':
        call(probe, b.run, Trigger.Request())
    rep.check('режим стенда RUN (stand/state, transient_local)',
              wait_for(lambda: probe.string('stand/state') == 'RUN', 5.0), 'получено: %s' % probe.string('stand/state'))
    wait_for(lambda: b.arm() is not None and None not in b.arm(), 5.0)
    if args.stuck:
        return check_stuck(b, rep)

    # ---- joint_states
    wait_for(lambda: len(probe.js_times) > 60, 5.0)
    with probe.lock:
        times, stamps, js = list(probe.js_times), list(probe.js_stamps), dict(probe.js or {})
    rate = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 2 else 0.0
    rep.check('joint_states: все 6 суставов по именам', set(ALL) <= set(js), 'имена: %s' % sorted(js))
    rep.check('joint_states: частота не ниже 40 Гц', rate >= 40.0, '%.1f Гц' % rate)
    mono = all(b2 > a for a, b2 in zip(stamps, stamps[1:]))
    age = time.time() - stamps[-1] if stamps else math.inf
    rep.check('joint_states: метки возрастают и не из будущего', mono and -0.01 < age < 0.5,
              'возраст последней метки %.1f мс' % (age * 1e3))

    # ---- команды по топику
    q0 = b.arm()
    b.command(['joint2'], [0.30])
    moved = wait_for(lambda: abs(b.arm()[1] - 0.30) < 0.06, 5.0)
    q1 = b.arm()
    rep.check('частичная команда двигает только свой сустав', moved and abs(q1[0] - q0[0]) < 0.03,
              'joint2 = %.3f, joint1: %.3f -> %.3f' % (q1[1], q0[0], q1[0]))
    b.command(['elbow'], [1.0])
    time.sleep(0.5)
    q2 = b.arm()
    rep.check('команда с неизвестным именем не исполняется', max(abs(a - c) for a, c in zip(q1, q2)) < 0.03)
    b.command(['joint4'], [2.5])
    time.sleep(0.5)
    rep.check('команда за пределом сустава не исполняется', abs(b.arm()[3] - q2[3]) < 0.03)

    if not args.lab3:
        check_trajectories(b, rep)

    # смена режима прерывает исполнение и снимает поток уставок
    ok_hold = call(probe, b.hold, Trigger.Request())
    rep.check('stand/hold: режим HOLD', ok_hold is not None and ok_hold.success and
              wait_for(lambda: probe.string('stand/state') == 'HOLD', 3.0))
    time.sleep(0.5)
    rep.check('в HOLD сторожевой таймер не срабатывает', probe.string('stand/state') == 'HOLD')
    ok_run = call(probe, b.run, Trigger.Request())
    rep.check('stand/run: снова RUN и удержание без рывка', ok_run is not None and ok_run.success and
              wait_for(lambda: probe.string('stand/state') == 'RUN', 3.0))
    time.sleep(0.5)
    rep.check('RUN держится (поток уставок идёт)', probe.string('stand/state') == 'RUN',
              'режим %s' % probe.string('stand/state'))
    return rep.summary()


def check_trajectories(b, rep):
    """ЛР4-А: интерполяция, коды отказов, арбитраж, вытеснение, отмена."""
    # ---- траектории
    start = b.arm()
    mid = [0.3, 0.4, 0.5, 0.6, -0.3]
    end = [0.0, 0.2, 0.9, 0.9, 0.0]
    for with_vel, kind in ((False, 'только положения'), (True, 'положения и скорости')):
        start = b.arm()
        goal = make_goal([(0.0, start), (1.5, mid), (3.0, end)], with_vel=with_vel)
        status, res = b.result(b.send(goal), 15.0)
        err = max(abs(a - c) for a, c in zip(b.arm(), end))
        rep.check('траектория (%s): SUCCESSFUL, цель достигнута' % kind,
                  res is not None and res.error_code == FJT.SUCCESSFUL and err < 0.08,
                  'код %s «%s», ошибка %.3f рад' % (getattr(res, 'error_code', None),
                                                  getattr(res, 'error_string', ''), err))

    cur = b.arm()
    for names, pts, code, what in (
            (ARM[:4] + ['wrist9'], [(0.0, cur), (1.0, cur)], FJT.INVALID_JOINTS, 'чужой сустав'),
            (ARM, [(0.0, cur), (1.0, [0, 0, 0, 2.5, 0])], FJT.INVALID_GOAL, 'точка за пределом'),
            (ARM, [(0.0, [c + 0.5 for c in cur]), (1.0, cur)], FJT.INVALID_GOAL, 'начало далеко от факта'),
            (ARM, [(1.0, cur), (0.5, cur)], FJT.INVALID_GOAL, 'время убывает')):
        status, res = b.result(b.send(make_goal(pts, names=names)), 5.0)
        rep.check('отказ с типизированным кодом: %s' % what, res is not None and res.error_code == code,
                  'код %s «%s»' % (getattr(res, 'error_code', None), getattr(res, 'error_string', '')))

    # арбитраж: команда по топику во время траектории игнорируется
    cur = b.arm()
    target = [0.4, 0.3, 0.6, 0.6, 0.4]
    gh = b.send(make_goal([(0.0, cur), (3.0, target)], with_vel=True))
    time.sleep(1.0)
    b.command(ARM, [-0.8, -0.5, 0.0, 0.0, -0.8])
    status, res = b.result(gh, 15.0)
    rep.check('арбитраж: топик не вмешивается в траекторию', res is not None and res.error_code == FJT.SUCCESSFUL
              and max(abs(a - c) for a, c in zip(b.arm(), target)) < 0.08,
              'код %s «%s»' % (getattr(res, 'error_code', None), getattr(res, 'error_string', '')))

    # вытеснение: вторая цель прерывает первую
    cur = b.arm()
    gh1 = b.send(make_goal([(0.0, cur), (4.0, [0.0, 0.0, 0.3, 0.3, 0.0])]))
    time.sleep(1.0)
    now = b.arm()
    final = [-0.3, 0.2, 0.6, 0.6, -0.3]
    gh2 = b.send(make_goal([(0.0, now), (2.0, final)]))
    s1, r1 = b.result(gh1, 10.0)
    s2, r2 = b.result(gh2, 10.0)
    rep.check('вытеснение: первая цель прервана, вторая исполнена',
              s1 == 6 and r2 is not None and r2.error_code == FJT.SUCCESSFUL,       # 6 = ABORTED
              'статус первой %s «%s», код второй %s' % (s1, getattr(r1, 'error_string', ''),
                                                       getattr(r2, 'error_code', None)))

    # отмена: удержание текущего положения
    cur = b.arm()
    gh = b.send(make_goal([(0.0, cur), (4.0, [0.5, 0.5, 0.5, 0.5, 0.5])]))
    time.sleep(1.2)
    wait_future(gh.cancel_goal_async(), 5.0)
    s, r = b.result(gh, 5.0)
    time.sleep(0.4)
    qa = b.arm()
    time.sleep(0.5)
    qb = b.arm()
    rep.check('отмена клиентом: CANCELED и удержание', s == 5 and max(abs(a - c) for a, c in zip(qa, qb)) < 0.02,
              'статус %s, дрейф за 0,5 с %.3f рад' % (s, max(abs(a - c) for a, c in zip(qa, qb))))


def check_stuck(b, rep):
    """Стенд запущен с stuck:=J@T. Размашистые движения туда-обратно, пока сустав не заклинит:
    отказ обязан прийти ВО ВРЕМЯ движения кодом PATH_TOLERANCE_VIOLATED, после — удержание."""
    poses = ([0.5, 0.6, 0.9, 0.9, 0.5], [-0.5, -0.3, 0.0, 0.0, -0.5])
    t0 = time.monotonic()
    res, k, t_goal, duration_s = None, 0, 0.0, 3.0
    while time.monotonic() - t0 < 90.0:
        cur = b.arm()
        t_goal = time.monotonic()
        _, res = b.result(b.send(make_goal([(0.0, cur), (duration_s, poses[k % 2])], with_vel=True)), 20.0)
        k += 1
        if res is None or res.error_code != FJT.SUCCESSFUL:
            break
    elapsed = time.monotonic() - t_goal
    qa = b.arm()
    time.sleep(0.5)
    qb = b.arm()
    rep.check('заклинивание: PATH_TOLERANCE_VIOLATED во время движения',
              res is not None and res.error_code == FJT.PATH_TOLERANCE_VIOLATED and elapsed < duration_s + 0.2,
              'код %s «%s», через %.2f с после начала' % (getattr(res, 'error_code', None),
                                                         getattr(res, 'error_string', ''), elapsed))
    rep.check('заклинивание: удержание после отказа', max(abs(a - c) for a, c in zip(qa, qb)) < 0.03,
              'дрейф за 0,5 с %.3f рад' % max(abs(a - c) for a, c in zip(qa, qb)))
    return rep.summary()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stuck', action='store_true', help='стенд запущен с stuck:=J@T')
    ap.add_argument('--lab3', action='store_true', help='только проверки ЛР3 (без траекторий ЛР4-А)')
    args, _ = ap.parse_known_args()
    run_main(lambda probe: checks(probe, args), 'check_bridge')


if __name__ == '__main__':
    main()

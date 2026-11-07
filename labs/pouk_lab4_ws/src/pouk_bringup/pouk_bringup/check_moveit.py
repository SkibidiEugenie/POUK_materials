# -*- coding: utf-8 -*-
"""Самопроверка контура «MoveIt -> мост -> стенд» (ЛР4-А, шаги 6–7).

    ros2 launch pouk_bringup lab4a.launch.py sim:=true rviz:=false   # терминал 1
    ros2 run pouk_bringup check_moveit --variant 3                    # терминал 2
    ros2 run pouk_bringup check_moveit --a 80 90 30 --b 80 -90 30 --obstacle 115 0 45 50 50 80

Проверяется: план исполняется мостом (а не «фиктивным» контроллером); позиционная цель
для 5-звенной руки решается; препятствие варианта действительно перекрывает прямой путь
A -> B, а план его обходит; цель внутри препятствия отклоняется планировщиком, а не
исполнителем. Координаты — миллиметры в системе base_link.
"""
import argparse
import math
import time

from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes, PositionConstraint
from shape_msgs.msg import SolidPrimitive

from pouk_link import kinematics as K

from . import scene
from .checklib import Report, run_main, wait_for, wait_future

ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
CODES = {v: k for k, v in MoveItErrorCodes.__dict__.items() if k.isupper() and isinstance(v, int)}


def variant_geometry(k, z, rot_deg, spread_deg, obs_k, w, h):
    """Точки A и B на радиусе k по обе стороны от препятствия, стоящего на плите."""
    ra, rb, ro = math.radians(rot_deg + spread_deg), math.radians(rot_deg - spread_deg), math.radians(rot_deg)
    a = (k * math.cos(ra), k * math.sin(ra), z)
    b = (k * math.cos(rb), k * math.sin(rb), z)
    obstacle = (obs_k * math.cos(ro), obs_k * math.sin(ro), 5.0 + h / 2.0, w, w, h)
    return a, b, obstacle


# варианты ЛР4-А: (k, z, поворот, полураствор, радиус препятствия, ширина, высота), мм и градусы
VARIANTS = {
    1: (120, 30, 0, 48, 115, 50, 80),
    2: (115, 35, 20, 45, 110, 45, 80),
    3: (125, 25, -20, 50, 120, 50, 70),
    4: (110, 40, 35, 42, 105, 40, 85),
    5: (130, 30, -35, 45, 125, 45, 75),
    6: (120, 45, 10, 55, 115, 45, 75),
    7: (105, 30, -10, 60, 100, 40, 80),
    8: (125, 35, 40, 40, 120, 45, 70),
}


def request(goal_constraints, execute=True):
    goal = MoveGroup.Goal()
    r = goal.request
    r.group_name = 'arm'
    r.num_planning_attempts = 3
    r.allowed_planning_time = 5.0
    r.max_velocity_scaling_factor = 0.5
    r.max_acceleration_scaling_factor = 0.5
    r.workspace_parameters.header.frame_id = 'base_link'
    r.workspace_parameters.min_corner.x = r.workspace_parameters.min_corner.y = -0.5
    r.workspace_parameters.min_corner.z = -0.2
    r.workspace_parameters.max_corner.x = r.workspace_parameters.max_corner.y = 0.5
    r.workspace_parameters.max_corner.z = 0.6
    r.goal_constraints = [goal_constraints]
    goal.planning_options.plan_only = not execute
    goal.planning_options.planning_scene_diff.is_diff = True
    goal.planning_options.planning_scene_diff.robot_state.is_diff = True
    return goal


def joint_goal(q, tol=0.01):
    c = Constraints()
    c.joint_constraints = [JointConstraint(joint_name=n, position=float(v), tolerance_above=tol,
                                           tolerance_below=tol, weight=1.0) for n, v in zip(ARM, q)]
    return c


def position_goal(x, y, z, radius=0.005):
    pc = PositionConstraint()
    pc.header.frame_id = 'base_link'
    pc.link_name = 'tool0'
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z
    pose.orientation.w = 1.0
    pc.constraint_region = BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[radius])],
                                          primitive_poses=[pose])
    pc.weight = 1.0
    return Constraints(position_constraints=[pc])


def straight_path_blocked(qa, qb, box, inflate=0.012, steps=100):
    """Пересекает ли прямой путь в пространстве суставов препятствие (ПЗК кончика схвата).
    При линейной интерполяции сумма α2+α3+α4 остаётся равной π, схват вертикален и ПЗК верна."""
    cx, cy, cz, sx, sy, sz = box
    for i in range(steps + 1):
        s = i / float(steps)
        x, y, z = K.fk([a + (b - a) * s for a, b in zip(qa, qb)])
        if abs(x - cx) <= sx / 2 + inflate and abs(y - cy) <= sy / 2 + inflate and z <= cz + sz / 2 + inflate:
            return True
    return False


def checks(probe, args):
    rep = Report('Контур MoveIt')
    if args.variant:
        a_mm, b_mm, box_mm = variant_geometry(*VARIANTS[args.variant])
    else:
        a_mm, b_mm, box_mm = tuple(args.a), tuple(args.b), tuple(args.obstacle)
    a_m = [v / 1000.0 for v in a_mm]
    b_m = [v / 1000.0 for v in b_mm]
    box = [v / 1000.0 for v in box_mm]
    print('A = (%.0f, %.0f, %.0f) мм, B = (%.0f, %.0f, %.0f) мм, препятствие: центр (%.0f, %.0f, %.0f), '
          'размер %.0f×%.0f×%.0f мм' % (a_mm + b_mm + box_mm), flush=True)
    ik_a, ik_b = K.ik(*a_m), K.ik(*b_m)
    if not rep.check('точки A и B достижимы (ОЗК приложения А)', ik_a.ok and ik_b.ok,
                     '%s %s' % (ik_a.reason, ik_b.reason)):
        return rep.summary()
    qa, qb = ik_a.q, ik_b.q
    rep.check('препятствие перекрывает прямой путь A -> B', straight_path_blocked(qa, qb, box))

    client = ActionClient(probe, MoveGroup, 'move_action')
    rep.check('move_group доступен (действие move_action)', client.wait_for_server(timeout_sec=30.0))
    rep.check('стенд в RUN', wait_for(lambda: probe.string('stand/state') == 'RUN', 10.0))
    wait_for(lambda: probe.q(ARM) is not None, 5.0)

    def move(constraints, execute=True, timeout=40.0, attempts=3):
        # выборочный планировщик стохастичен: отказ планирования повторяют (как replan в MoveIt),
        # а отказ исполнения — нет: он означает нарушение контракта исполнителем
        res = None
        for _ in range(attempts):
            gh = wait_future(client.send_goal_async(request(constraints, execute and not args.plan_only)), 10.0)
            if gh is None or not gh.accepted:
                return None
            wrapped = wait_future(gh.get_result_async(), timeout)
            res = None if wrapped is None else wrapped.result
            if res is None or res.error_code.val not in (MoveItErrorCodes.PLANNING_FAILED,
                                                         MoveItErrorCodes.INVALID_MOTION_PLAN):
                break
        return res

    def code(res):
        return 'нет ответа' if res is None else CODES.get(res.error_code.val, res.error_code.val)

    def err_to(q):
        return 0.0 if args.plan_only else max(abs(x - y) for x, y in zip(probe.q(ARM), q))

    scene.apply(probe, [scene.box_object('obstacle', (0, 0, 0), (0, 0, 0), remove=True)])
    res = move(joint_goal([0.0] * 5))
    time.sleep(0.5)
    rep.check('план в home исполнен на стенде', res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS
              and err_to([0.0] * 5) < 0.06, '%s, ошибка %.3f рад' % (code(res), err_to([0.0] * 5)))

    res = move(joint_goal(qa))
    time.sleep(0.5)
    rep.check('план в точку A исполнен', res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS
              and err_to(qa) < 0.06, '%s, ошибка %.3f рад' % (code(res), err_to(qa)))
    if res is not None and res.planned_trajectory.joint_trajectory.points:
        pt = res.planned_trajectory.joint_trajectory.points[-1]
        rep.check('план несёт скорости и ускорения (интерполяция C²)',
                  len(pt.velocities) == 5 and len(pt.accelerations) == 5)

    if not args.plan_only:
        res = move(position_goal(0.13, 0.0, 0.05))
        time.sleep(0.5)
        # ориентация позиционной целью не задаётся — ПЗК приложения А здесь неприменима,
        # поэтому контроль по коду результата, положение смотрят в RViz
        rep.check('позиционная цель для 5-звенной руки решена (position_only_ik)',
                  res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS, code(res))
        move(joint_goal(qa))

    ok = scene.apply(probe, [scene.box_object('obstacle', box[:3], box[3:])])
    rep.check('препятствие добавлено в сцену', ok)
    res = move(joint_goal(qb))
    n_pts = len(res.planned_trajectory.joint_trajectory.points) if res is not None else 0
    time.sleep(0.5)
    rep.check('с препятствием: обходной план A -> B исполнен', res is not None and res.error_code.val == 1
              and err_to(qb) < 0.06, '%s, точек в плане %d' % (code(res), n_pts))
    inside = K.ik(box[0], box[1], max(0.02, box[2] - box[5] / 2 + 0.02))
    if inside.ok:
        res = move(joint_goal(inside.q), execute=False, attempts=1)
        rep.check('цель внутри препятствия отклонена планировщиком',
                  res is not None and res.error_code.val != MoveItErrorCodes.SUCCESS, code(res))
    scene.apply(probe, [scene.box_object('obstacle', (0, 0, 0), (0, 0, 0), remove=True)])
    res = move(joint_goal([0.0] * 5))
    rep.check('возврат в home', res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS, code(res))
    return rep.summary()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--variant', type=int, choices=sorted(VARIANTS), help='номер варианта бригады')
    ap.add_argument('--a', type=float, nargs=3, default=[80.0, 90.0, 30.0], metavar=('X', 'Y', 'Z'))
    ap.add_argument('--b', type=float, nargs=3, default=[80.0, -90.0, 30.0], metavar=('X', 'Y', 'Z'))
    ap.add_argument('--obstacle', type=float, nargs=6, default=[115.0, 0.0, 45.0, 50.0, 50.0, 80.0],
                    metavar=('CX', 'CY', 'CZ', 'SX', 'SY', 'SZ'))
    ap.add_argument('--plan-only', action='store_true', help='только планирование, без исполнения')
    args, _ = ap.parse_known_args()
    run_main(lambda probe: checks(probe, args), 'check_moveit')


if __name__ == '__main__':
    main()

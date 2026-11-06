# -*- coding: utf-8 -*-
"""Самопроверка координатора (ЛР3, шаги 5–6): сценарии и отказы через интерфейсы ROS 2.

    ros2 launch pouk_bringup lab3.launch.py sim:=true rviz:=false   # терминал 1
    ros2 run pouk_bringup check_coordinator                          # терминал 2
"""
import time

from std_srvs.srv import Trigger

from pouk_interfaces.srv import Circle, MoveTo
from pouk_link import kinematics as K

from .checklib import Report, call, run_main, wait_for

ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']


def checks(probe):
    rep = Report('Координатор')
    probe.watch_string('coordinator/state')
    probe.watch_string('coordinator/status')
    srv = {name: probe.create_client(Trigger, 'coordinator/' + name) for name in ('home', 'stop', 'reset')}
    move = probe.create_client(MoveTo, 'coordinator/move_to')
    circle = probe.create_client(Circle, 'coordinator/circle')
    run = probe.create_client(Trigger, 'stand/run')
    hold = probe.create_client(Trigger, 'stand/hold')

    def state():
        return probe.string('coordinator/state')

    def trig(name):
        return call(probe, srv[name], Trigger.Request())

    def move_to(x, y, z):
        return call(probe, move, MoveTo.Request(x=x, y=y, z=z, vmax=0.0))

    rep.check('состояние наблюдаемо (coordinator/state, transient_local)',
              wait_for(lambda: state() is not None, 10.0), 'состояние %s' % state())
    wait_for(lambda: probe.string('stand/state') == 'RUN', 10.0)
    if state() == 'FAULT':
        trig('reset')

    r = trig('home')
    rep.check('home принят, HOMING', r is not None and r.success and wait_for(lambda: state() in ('HOMING', 'IDLE'), 2.0))
    r2 = trig('home')
    rep.check('повторный home идемпотентен', r2 is not None and r2.success, r2.message if r2 else 'нет ответа')
    rep.check('исходное достигнуто: IDLE', wait_for(lambda: state() == 'IDLE', 15.0), probe.string('coordinator/status'))

    r = move_to(0.40, 0.0, 0.03)
    rep.check('недостижимая точка отклонена с причиной', r is not None and not r.accepted and state() == 'IDLE',
              r.message if r else 'нет ответа')

    target = (0.12, 0.03, 0.03)
    r = move_to(*target)
    ok = r is not None and r.accepted and wait_for(lambda: state() == 'MOVE_POINT', 2.0) is not None
    reached = wait_for(lambda: state() == 'IDLE', 15.0)
    x, y, z = K.fk(probe.q(ARM))
    err = ((x - target[0]) ** 2 + (y - target[1]) ** 2 + (z - target[2]) ** 2) ** 0.5
    rep.check('move_to: точка достигнута (ПЗК по joint_states)', ok and reached and err < 0.012,
              'ошибка %.1f мм' % (err * 1e3))

    r = call(probe, circle, Circle.Request(cx=0.10, cy=0.0, cz=0.03, r=0.03, n=8, laps=1), 5.0)
    rep.check('circle принят: CIRCLE', r is not None and r.accepted and wait_for(lambda: state() == 'CIRCLE', 2.0),
              r.message if r else 'нет ответа')
    r = move_to(*target)
    rep.check('move_to во время CIRCLE отклонён', r is not None and not r.accepted, r.message if r else '')
    rep.check('окружность пройдена: IDLE', wait_for(lambda: state() == 'IDLE', 60.0), probe.string('coordinator/status'))

    move_to(0.10, -0.05, 0.05)
    time.sleep(0.3)
    r = trig('stop')
    rep.check('stop во время движения: IDLE', r is not None and r.success and wait_for(lambda: state() == 'IDLE', 2.0))

    # отказ: стенд выходит из RUN во время движения
    move_to(0.12, 0.05, 0.03)
    time.sleep(0.3)
    call(probe, hold, Trigger.Request())
    rep.check('стенд вышел из RUN во время движения: FAULT', wait_for(lambda: state() == 'FAULT', 3.0),
              probe.string('coordinator/status'))
    r = trig('home')
    rep.check('в FAULT запросы движения отклоняются', r is not None and not r.success)
    r = trig('reset')
    rep.check('reset при неготовом стенде отклонён', r is not None and not r.success and state() == 'FAULT',
              r.message if r else '')
    call(probe, run, Trigger.Request())
    wait_for(lambda: probe.string('stand/state') == 'RUN', 3.0)
    time.sleep(0.3)
    r = trig('reset')
    rep.check('reset после устранения причины: IDLE', r is not None and r.success and state() == 'IDLE')
    return rep.summary()


def main():
    run_main(checks, 'check_coordinator')


if __name__ == '__main__':
    main()

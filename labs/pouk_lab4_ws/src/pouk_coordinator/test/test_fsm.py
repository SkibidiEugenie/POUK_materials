# -*- coding: utf-8 -*-
"""ЛР3, шаг 5: автомат координатора без ROS и без стенда."""
import pytest

from pouk_coordinator import fsm
from pouk_coordinator.fsm import CIRCLE, FAULT, HOMING, IDLE, MOVE_POINT, Coordinator, next_state
from pouk_link import kinematics as K


def at(q):
    return list(q[:5]) + [0.0]


def test_fault_reachable_from_every_state_and_only_reset_leaves_it():
    for s in fsm.STATES:
        assert next_state(s, 'fault') == FAULT
    for e in fsm.EVENTS:
        if e not in ('reset', 'fault'):
            assert next_state(FAULT, e) is None, e
    assert next_state(FAULT, 'reset') == IDLE


def test_stop_defined_in_every_motion_state():
    for s in (HOMING, MOVE_POINT, CIRCLE):
        assert next_state(s, 'stop') == IDLE


def test_home_is_idempotent_and_reaches_idle():
    c = Coordinator(reach_timeout=5.0)
    q0 = [0.3, 0.2, 0.1, 0.0, 0.3, 0.0]
    ok, _ = c.request_home(0.0, True)
    assert ok and c.state == HOMING and c.target == [0.0] * 6
    deadline = c._deadline
    ok, msg = c.request_home(2.0, True)                  # повтор через 2 с
    assert ok and c.state == HOMING and c._deadline == deadline and 'уже' in msg
    c.tick(2.5, q0, True)
    assert c.state == HOMING
    c.tick(3.0, [0.01, -0.02, 0.0, 0.03, 0.0, 0.5], True)  # схват не учитывается
    assert c.state == IDLE


def test_move_to_unreachable_rejected_without_state_change():
    c = Coordinator()
    ok, msg = c.request_move_to(0.4, 0.0, 0.03, 0.0, 0.0, True)
    assert not ok and 'недостижима' in msg and c.state == IDLE and c.target is None


def test_move_to_same_point_does_not_restart_timeout():
    c = Coordinator(reach_timeout=5.0)
    assert c.request_move_to(0.12, 0.0, 0.03, 0.0, 0.0, True)[0]
    d = c._deadline
    ok, msg = c.request_move_to(0.12, 0.0, 0.03, 0.0, 3.0, True)
    assert ok and c._deadline == d and 'уже' in msg
    assert c.request_move_to(0.10, 0.02, 0.03, 0.0, 3.0, True)[0]   # новая цель — новый срок
    assert c._deadline == 8.0


def test_requests_rejected_when_stand_not_ready_or_wrong_state():
    c = Coordinator()
    assert not c.request_home(0.0, False)[0] and c.state == IDLE
    assert c.request_circle(0.1, 0.0, 0.03, 0.03, 8, 1, 0.0, True)[0] and c.state == CIRCLE
    ok, msg = c.request_move_to(0.12, 0.0, 0.03, 0.0, 0.0, True)
    assert not ok and 'недопустимо' in msg and c.state == CIRCLE


def test_stand_leaving_run_or_timeout_leads_to_fault_with_reason():
    c = Coordinator(reach_timeout=2.0)
    c.request_home(0.0, True)
    c.tick(0.5, [0.5] * 6, False, 'стенд в режиме WD_STOP')
    assert c.state == FAULT and 'WD_STOP' in c.message
    assert not c.request_home(1.0, True)[0]              # из отказа — только reset
    assert not c.request_reset(1.0, False)[0] and c.state == FAULT
    assert c.request_reset(1.0, True)[0] and c.state == IDLE

    c.request_home(2.0, True)
    c.tick(4.5, [0.0, 0.0, 0.5, 0.0, 0.0, 0.0], True)
    assert c.state == FAULT and 'сустав 3' in c.message


def test_circle_goes_through_all_points_and_laps():
    c = Coordinator()
    assert c.request_circle(0.1, 0.0, 0.03, 0.03, 4, 2, 0.0, True)[0]
    visited = 0
    t = 0.0
    while c.state == CIRCLE and visited < 20:
        t += 0.1
        c.tick(t, at(c.target), True)                    # стенд мгновенно в уставке
        visited += 1
    assert c.state == IDLE and visited == 8 and 'пройдена' in c.message


def test_circle_rejects_unreachable_point():
    c = Coordinator()
    ok, msg = c.request_circle(0.15, 0.0, 0.03, 0.06, 12, 1, 0.0, True)
    assert not ok and c.state == IDLE and 'точка' in msg


def test_stop_holds_actual_position():
    c = Coordinator()
    c.request_move_to(0.12, 0.0, 0.03, 0.0, 0.0, True)
    q = [0.1, 0.2, 0.3, 0.4, 0.1, 0.0]
    assert c.request_stop(1.0, q)[0] and c.state == IDLE and c.target == q

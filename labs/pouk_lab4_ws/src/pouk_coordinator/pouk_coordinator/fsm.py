# -*- coding: utf-8 -*-
"""Координатор стенда как конечный автомат (ЛР3; пособие, §4.3 и §4.8).

Состояния:  IDLE — ожидание; HOMING — наведение в исходное; MOVE_POINT — движение к точке;
            CIRCLE — обход окружности по точкам; FAULT — отказ (выход только по reset).
События:    запросы оператора home / move_to / circle / stop / reset;
            внутренние reached (цель достигнута), done (окружность пройдена),
            fault (стенд не в RUN, нет телеметрии, тайм-аут достижения).

Три требования главы 4, которые проверяются тестами:
  * состояние наблюдаемо снаружи (узел публикует его, а не прячет в поле);
  * отказ достижим из ЛЮБОГО состояния;
  * повторная команда идемпотентна: второй home не перезапускает наведение,
    повторный move_to в ту же точку не сбрасывает тайм-аут.

Ядро не зависит от ROS: вход — время, фактические углы и признак готовности стенда,
выход — уставка суставов (target) и сообщение (message). Узел node.py лишь переводит
это в топики и сервисы.
"""
import math

from pouk_link import kinematics as K

IDLE, HOMING, MOVE_POINT, CIRCLE, FAULT = 'IDLE', 'HOMING', 'MOVE_POINT', 'CIRCLE', 'FAULT'
STATES = (IDLE, HOMING, MOVE_POINT, CIRCLE, FAULT)
EVENTS = ('home', 'move_to', 'circle', 'stop', 'reset', 'reached', 'done', 'fault')

# >>> STUDENT ЛР3-шаг5: таблица переходов по вашей диаграмме состояний (рисуется ДО кода).
# >>> Ключ — (состояние, событие), значение — новое состояние. Пары, которых нет в таблице,
# >>> означают «событие в этом состоянии недопустимо» (запрос отклоняется с объяснением).
# >>> Переход по событию fault из любого состояния задан отдельно в next_state().
TRANSITIONS = {}  # TODO: заполнить по диаграмме
# <<< STUDENT


def next_state(state, event):
    """Новое состояние или None, если событие в этом состоянии недопустимо."""
    if event == 'fault':
        return FAULT                       # отказ достижим из любого состояния — одним правилом
    return TRANSITIONS.get((state, event))


class Coordinator(object):
    def __init__(self, reach_eps=0.08, reach_timeout=8.0, vmax=0.8, home=(0.0,) * 6,
                 circle_defaults=(0.100, 0.0, 0.030, 0.030, 20, 1)):
        self.reach_eps = reach_eps
        self.reach_timeout = reach_timeout
        self.vmax = vmax
        self.home = list(home)
        self.circle_defaults = circle_defaults
        self.state = IDLE
        self.message = 'ожидание'
        self.target = None                 # уставка на стенд: 6 углов (рад)
        self.target_vmax = vmax
        self.gripper = 0.0
        self._deadline = None
        self._points = []                  # уставки окружности
        self._index = 0
        self._laps = 0
        self._lap = 0
        self.changed = True                # признак для узла: опубликовать состояние

    # ---------------------------------------------------------------- переходы
    def _go(self, event, message):
        new = next_state(self.state, event)
        if new is None:
            return False
        if new != self.state or message != self.message:
            self.changed = True
        self.state, self.message = new, message
        return True

    def _reject(self, event):
        return False, 'событие %s недопустимо в состоянии %s' % (event, self.state)

    def _set_target(self, q, now, vmax=None):
        self.target = list(q[:5]) + [self.gripper]
        self.target_vmax = vmax if vmax else self.vmax
        self._deadline = now + self.reach_timeout

    # ---------------------------------------------------------------- запросы оператора
    def request_home(self, now, stand_ok):
        if not stand_ok and next_state(self.state, 'home') is not None:
            return False, 'стенд не готов (нужен режим RUN)'
        if self.state == HOMING:
            return True, 'наведение уже выполняется'          # идемпотентно: цель и срок прежние
        if not self._go('home', 'наведение в исходное'):
            return self._reject('home')
        self._set_target(self.home, now)
        return True, self.message

    def request_move_to(self, x, y, z, vmax, now, stand_ok):
        if next_state(self.state, 'move_to') is None:
            return self._reject('move_to')
        if not stand_ok:
            return False, 'стенд не готов (нужен режим RUN)'
        r = K.ik(x, y, z)
        if not r.ok:
            return False, 'точка (%.3f, %.3f, %.3f) недостижима: %s' % (x, y, z, r.reason)
        q = K.with_gripper(r.q, self.gripper)
        if self.state == MOVE_POINT and self.target is not None and \
                max(abs(a - b) for a, b in zip(q, self.target)) < 1e-6:
            return True, 'движение к этой точке уже выполняется'
        self._go('move_to', 'движение к (%.3f, %.3f, %.3f)' % (x, y, z))
        self._set_target(q, now, vmax)
        return True, self.message

    def request_circle(self, cx, cy, cz, r, n, laps, now, stand_ok):
        if next_state(self.state, 'circle') is None:
            return self._reject('circle')
        if not stand_ok:
            return False, 'стенд не готов (нужен режим RUN)'
        dcx, dcy, dcz, dr, dn, dlaps = self.circle_defaults
        n = n or dn
        laps = laps or dlaps
        if r <= 0:
            cx, cy, cz, r = dcx, dcy, dcz, dr
        if n < 3:
            return False, 'на окружности нужно не меньше 3 точек'
        points = []
        for i, (x, y, z) in enumerate(K.circle_points(cx, cy, cz, r, n)):
            ik = K.ik(x, y, z)
            if not ik.ok:
                return False, 'точка %d окружности недостижима: %s' % (i, ik.reason)
            points.append(K.with_gripper(ik.q, self.gripper))
        self._points, self._index, self._laps, self._lap = points, 0, laps, 0
        self._go('circle', 'окружность R = %.0f мм, N = %d: точка 1/%d, круг 1/%d'
                 % (r * 1000, n, n, laps))
        self._set_target(points[0], now)
        return True, self.message

    def request_stop(self, now, q_actual):
        if not self._go('stop', 'остановлено оператором'):
            return self._reject('stop')
        if q_actual is not None:
            self._set_target(q_actual, now)                    # удерживать там, где стоим
        self._deadline = None
        return True, self.message

    def request_reset(self, now, stand_ok):
        if self.state != FAULT:
            return self._reject('reset')
        if not stand_ok:
            return False, 'причина отказа не устранена: стенд не в RUN'
        self._go('reset', 'отказ сброшен')
        self.target, self._deadline = None, None
        return True, self.message

    # ---------------------------------------------------------------- такт
    def reached(self, q_actual):
        return self.target is not None and q_actual is not None and \
            all(abs(q_actual[j] - self.target[j]) <= self.reach_eps for j in range(5))

    def tick(self, now, q_actual, stand_ok, stand_reason=''):
        """Периодический такт: контроль готовности стенда, достижения и сроков."""
        if self.state in (HOMING, MOVE_POINT, CIRCLE):
            if not stand_ok:
                self._go('fault', 'отказ: %s' % (stand_reason or 'стенд вышел из RUN'))
                return
            if self.reached(q_actual):
                if self.state == CIRCLE:
                    self._advance_circle(now)
                else:
                    self._go('reached', 'цель достигнута')
                    self._deadline = None
                return
            if self._deadline is not None and now > self._deadline:
                worst = max(range(5), key=lambda j: abs(q_actual[j] - self.target[j])) if q_actual else 0
                self._go('fault', 'отказ: тайм-аут %.1f с, сустав %d не дошёл на %.3f рад' % (
                    self.reach_timeout, worst + 1,
                    abs(q_actual[worst] - self.target[worst]) if q_actual else math.nan))

    def _advance_circle(self, now):
        self._index += 1
        if self._index >= len(self._points):
            self._index = 0
            self._lap += 1
            if self._lap >= self._laps:
                self._go('done', 'окружность пройдена: %d кругов' % self._laps)
                self._deadline = None
                return
        self.message = 'окружность: точка %d/%d, круг %d/%d' % (
            self._index + 1, len(self._points), self._lap + 1, self._laps)
        self.changed = True
        self._set_target(self._points[self._index], now)

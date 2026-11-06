# -*- coding: utf-8 -*-
"""Кинематика учебного стенда (приложение А к ЛР1, §А.3–А.5).

Соглашение: углы в радианах отсчитываются от вертикальной сборки (все приводы
в положении 512), схват смотрит вниз: α2 + α3 + α4 = π. Координаты — метры,
начало — ось основания на уровне крепления стенда.

Модуль — «ядро вычисления» без промежуточного ПО (пособие, §3.8.2): его
импортируют CLI ЛР2, координатор ЛР3 и тесты.
"""
import math
from collections import namedtuple

# длины условных звеньев, м (проверьте по своей сборке)
L1, L2, L3, L4 = 0.103, 0.1045, 0.079, 0.169
DEL = 0.00511327                     # рад в одном отсчёте AX-12A (≈300°/1024)
N_JOINTS = 6

# пределы суставов в отсчётах (каркас ЛР1); уточняются на шаге 2 ЛР1
JOINT_MIN_TICKS = (162, 262, 262, 362, 162, 312)
JOINT_MAX_TICKS = (862, 762, 892, 892, 862, 712)
# те же пределы в целых миллирадианах: round((t − 512) · DEL · 1000). Именно эти числа
# записаны в прошивке (mcu_logic.cpp, defaultConfig), в URDF и в виртуальном стенде —
# один источник истины, сравнение по целым числам без расхождений округления
JOINT_MIN_MRAD = (-1790, -1278, -1278, -767, -1790, -1023)
JOINT_MAX_MRAD = (1790, 1278, 1943, 1943, 1790, 1023)

IkResult = namedtuple('IkResult', 'ok q reason')


def ticks_to_rad(t):
    return (t - 512) * DEL


def rad_to_ticks(a):
    return 512 + int(round(a / DEL))


JOINT_MIN = tuple(v / 1000.0 for v in JOINT_MIN_MRAD)
JOINT_MAX = tuple(v / 1000.0 for v in JOINT_MAX_MRAD)


def in_limits(q, joints=range(N_JOINTS)):
    """Проверка пределов так же, как на МК: по целым миллирадианам после округления."""
    return all(JOINT_MIN_MRAD[j] <= int(round(q[j] * 1000.0)) <= JOINT_MAX_MRAD[j] for j in joints)


def ik(x, y, z):
    """ОЗК при вертикальном схвате. Возврат IkResult(ok, [q1..q5], причина отказа)."""
    k = math.hypot(x, y)
    z_r = z + (L4 - L1)
    d = math.hypot(k, z_r)
    if d > L2 + L3:
        return IkResult(False, None, 'цель дальше вытянутой руки: d = %.4f > %.4f м' % (d, L2 + L3))
    if d < abs(L2 - L3):
        return IkResult(False, None, 'цель в мёртвой зоне у оси: d = %.4f м' % d)
    c1 = (d * d + L2 * L2 - L3 * L3) / (2.0 * d * L2)
    c2 = (-d * d + L2 * L2 + L3 * L3) / (2.0 * L2 * L3)
    c1 = max(-1.0, min(1.0, c1))           # защита от округления на самой границе
    c2 = max(-1.0, min(1.0, c2))
    a1 = math.atan2(y, x) if k > 1e-9 else 0.0
    a2 = math.pi / 2 - (math.atan2(z_r, k) + math.acos(c1))
    a3 = math.pi - math.acos(c2)
    a4 = math.pi - (a2 + a3)
    q = [a1, a2, a3, a4, a1]
    bad = [j + 1 for j in range(5) if not (JOINT_MIN[j] <= q[j] <= JOINT_MAX[j])]
    if bad:
        return IkResult(False, q, 'нарушены пределы суставов %s' % bad)
    return IkResult(True, q, '')


def fk(q):
    """ПЗК по первым трём углам: (x, y, z) схвата в метрах."""
    a1, a2, a3 = q[0], q[1], q[2]
    k = L2 * math.sin(a2) + L3 * math.sin(a2 + a3)
    z = L1 + L2 * math.cos(a2) + L3 * math.cos(a2 + a3) - L4
    return k * math.cos(a1), k * math.sin(a1), z


def sagitta(r, n):
    """Стрелка сегмента: максимальное отклонение хорды от дуги (§А.5)."""
    return r * (1.0 - math.cos(math.pi / n))


def circle_points(cx, cy, cz, r, n):
    """N точек окружности в горизонтальной плоскости (м)."""
    return [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n), cz)
            for i in range(n)]


def with_gripper(q5, gripper=0.0):
    """Пять углов руки + угол схвата -> вектор шести уставок."""
    return list(q5[:5]) + [gripper]

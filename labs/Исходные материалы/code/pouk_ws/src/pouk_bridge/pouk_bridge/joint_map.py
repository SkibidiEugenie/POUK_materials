# -*- coding: utf-8 -*-
"""Соответствие имён суставов ROS и номеров приводов стенда.

Порядок имён в sensor_msgs/JointState ничем не гарантирован: разные издатели
(joint_state_publisher, MoveIt, координатор) перечисляют суставы по-разному, а
часть суставов может отсутствовать. Поэтому выборка ведётся по ИМЕНАМ, а не по
индексам (ошибка моста 2025 г.: position[0] уходил и в сустав 1, и в сустав 5).

Модуль не зависит от ROS и проверяется модульными тестами.
"""


class JointMapError(ValueError):
    pass


class JointMap(object):
    """names[i] — имя сустава ROS для привода ID i+1; sign и offset — калибровка:
    q_стенд = sign · q_ROS + offset."""

    def __init__(self, names, signs=None, offsets=None):
        self.names = list(names)
        n = len(self.names)
        if len(set(self.names)) != n:
            raise JointMapError('имена суставов повторяются: %s' % self.names)
        self.signs = list(signs) if signs else [1.0] * n
        self.offsets = list(offsets) if offsets else [0.0] * n
        if len(self.signs) != n or len(self.offsets) != n:
            raise JointMapError('длины joint_signs/joint_offsets не совпадают с joint_names')
        if any(s not in (1.0, -1.0) for s in self.signs):
            raise JointMapError('знак сустава должен быть +1 или -1')
        self.index = {name: i for i, name in enumerate(self.names)}

    def to_stand(self, names, positions, current=None):
        """Команда ROS -> вектор углов стенда в порядке приводов.

        Суставы, отсутствующие в команде, берутся из current (вектор стенда) — так
        команда только для схвата не сдвигает руку. Без current неполная команда —
        ошибка. Неизвестное имя — всегда ошибка: это признак чужой конфигурации.
        """
        if len(names) != len(positions):
            raise JointMapError('в команде %d имён и %d положений' % (len(names), len(positions)))
        # >>> STUDENT ЛР3-шаг2: выборка по именам. Для каждого имени найти номер привода
        # >>> в self.index (неизвестное имя -> JointMapError), пересчитать угол с учётом знака
        # >>> и смещения; отсутствующие суставы взять из current, а если current нет —
        # >>> JointMapError со списком отсутствующих имён.
        raise NotImplementedError('ЛР3, шаг 2: JointMap.to_stand')
        # <<< STUDENT

    def to_ros(self, q):
        """Вектор стенда -> (имена, положения) для sensor_msgs/JointState."""
        return list(self.names), [(q[i] - self.offsets[i]) * self.signs[i] for i in range(len(self.names))]

# -*- coding: utf-8 -*-
"""Добавить или убрать препятствие-параллелепипед в сцене MoveIt (ЛР4-А, шаг 7).

    ros2 run pouk_bringup add_obstacle --xyz 0.12 0 0.03 --size 0.04 0.04 0.06
    ros2 run pouk_bringup add_obstacle --id obstacle --remove
Координаты центра и размеры — метры в системе base_link.
"""
import argparse

import rclpy
from rclpy.node import Node

from . import scene
from .checklib import spin_in_background


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--id', default='obstacle')
    ap.add_argument('--xyz', type=float, nargs=3, default=[0.12, 0.0, 0.03])
    ap.add_argument('--size', type=float, nargs=3, default=[0.04, 0.04, 0.06])
    ap.add_argument('--remove', action='store_true')
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Node('add_obstacle')
    ex = spin_in_background(node)
    ok = scene.apply(node, [scene.box_object(args.id, args.xyz, args.size, remove=args.remove)])
    print(('убрано «%s»' if args.remove else 'добавлено «%s»') % args.id if ok else
          'не удалось: запущен ли move_group (lab4a.launch.py)?')
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()

# -*- coding: utf-8 -*-
"""Препятствия в планировочной сцене MoveIt (общий код add_obstacle и check_moveit)."""
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from .checklib import call


def box_object(name, xyz, size, frame='base_link', remove=False):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    if remove:
        co.operation = CollisionObject.REMOVE
        return co
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = [float(v) for v in xyz]
    pose.orientation.w = 1.0
    co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def apply(node, objects, timeout=5.0):
    """Применить изменения сцены синхронно (сервис move_group /apply_planning_scene)."""
    client = node.create_client(ApplyPlanningScene, 'apply_planning_scene')
    scene = PlanningScene(is_diff=True)
    scene.world.collision_objects = list(objects)
    res = call(node, client, ApplyPlanningScene.Request(scene=scene), timeout)
    return res is not None and res.success

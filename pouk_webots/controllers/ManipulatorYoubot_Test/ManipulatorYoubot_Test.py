from controller import Supervisor
import math
import sys
import numpy as np

TIME_STEP = 32
NEED_EPS = 0.01

robot = Supervisor()
robot_node = robot.getSelf()
timestep = int(robot.getBasicTimeStep())

if robot_node is None:
    print("ОШИБКА: Включите supervisor TRUE!")
    sys.exit(1)

translation_field = robot_node.getField("translation")
rotation_field = robot_node.getField("rotation")

def step():
    if robot.step(TIME_STEP) == -1:
        sys.exit(0)

joints = []
gripper = []
sensors = []

Z1 = 0.077 + 0.07          # arm1 (z=0.077) + arm2 (z=0.07) -> высота оси плеча
X1 = 0.033                 # вынос оси arm2 по x
L2, L3 = 0.155, 0.135      # arm3, arm4
L4 = 0.081 + 0.09          # arm5 + до основания пальцев (finger joint z=0.09)
LIM = {1: (-2.9496, 2.9496), 2: (-1.13446, 1.5708), 3: (-2.63545, 2.54818),
       4: (-1.78024, 1.78024), 5: (-2.92343, 2.92343)}

def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def trans(x, y, z):
    return [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]]


def fk(q):
    q1, q2, q3, q4, q5 = q
    chain = [
        trans(0, 0, 0.077), rot_z(q1),
        trans(X1, 0, 0.07), rot_y(-q2),     # ось -y => rot_y(-q)
        trans(0, 0, L2),    rot_y(-q3),
        trans(0, 0, L3),    rot_y(-q4),
        trans(0, 0, 0.081), rot_z(q5),
        trans(0, 0, 0.09),
    ]
    M = chain[0]
    for N in chain[1:]:
        M = matmul(M, N)
    return M


def wrap(a): return (a + np.pi) % (2 * np.pi) - np.pi

def ik(x, y, z, pitch=np.pi, roll=0.0):
    
    sols = []
    for s in (1, -1):                       # "вперёд" / "через спину"
        q1 = np.arctan2(y, x) if s == 1 else wrap(np.arctan2(y, x) + np.pi)
        r, c = s * np.hypot(x, y), s * pitch
        wz = z - Z1 - L4 * np.cos(c)        # запястье в плоскости (вертикаль, радиус)
        wr = r - X1 - L4 * np.sin(c)
        cos_t = (wz**2 + wr**2 - L2**2 - L3**2) / (2 * L2 * L3)
        if abs(cos_t) > 1: continue         # вне рабочей зоны
        for sign in (1, -1):                # локоть вверх/вниз
            t2 = sign * np.arccos(cos_t)
            a = np.arctan2(wr, wz) - np.arctan2(L3 * np.sin(t2), L2 + L3 * np.cos(t2))
            t3 = c - a - t2
            q = np.array([q1, -a, -t2, -t3, roll])   # минусы из-за оси -y
            if all(LIM[i + 1][0] <= q[i] <= LIM[i + 1][1] for i in range(5)):
                sols.append(q)
    return sols

def init_joints():
    for i in range(5):
        j = robot.getDevice(f"arm{i + 1}")
        k = robot.getDevice(f"arm{i+1}sensor")
        if j is None:
            print(f"ОШИБКА: Не найден сустав arm{i + 1}")
            return False
        if k is None:
            print(f"ОШИБКА: Не найден энкодер сустав arm{i + 1}")
            return False
        j.setVelocity(0.5)
        joints.append(j)
        k.enable(timestep)
        sensors.append(k)
    gripper.append(robot.getDevice('finger::right'))
    gripper.append(robot.getDevice('finger::left'))
    return True

def set_pose(angs):
    for i in range(5):
        joints[i].setPosition(angs[0][i])
 
def gripperState(state): 
    if state == 0:
        gripper[0].setPosition(0.025)
        gripper[1].setPosition(0.025)
    else:
        gripper[0].setPosition(0)
        gripper[1].setPosition(0)

def result_eps(M, target):
    eps = 0
    for i in range(3):
        eps += (M[i][3] - target[i]) ** 2
    eps = eps ** 0.5
    return eps

def createPoints(now, target):
    
    return res

def manipulatorMove(target):
    q = ik(*target, pitch=np.pi/2, roll=np.pi/2)
    angles = [sensors[i].getValue() for i in range(5)]
    M = fk(angles)
    set_pose(q)
    while (result_eps(M, target) > NEED_EPS):
        angles = [sensors[i].getValue() for i in range(5)]
        M = fk(angles)
        step()
    print("Done")
    
def transportationPose():
    target = (-0.25, 0, 0.2)
    manipulatorMove(target)
    
def takePlate(target):
    manipulatorMove(target)
    gripperState(1)
    

def main():
    if not init_joints():
        return
    step()

    start = translation_field.getSFVec3f()
    
    gripperState(0)
    takePlate((0, 0.3, 0.4))
    manipulatorMove((-0.25, 0, 0.4))
    transportationPose()

    while True:
        step()

main()
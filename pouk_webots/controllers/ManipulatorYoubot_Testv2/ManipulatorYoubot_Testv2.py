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

def fk_pose(q):
    M = np.array(fk(q))
    return M[:3, 3], M[:3, 2]          # позиция и ось z инструмента

def ik_num(x, y, z, pitch=np.pi, roll=0.0, q0=None,
           w_ori=0.3, tol_pos=1e-4, tol_ori=1e-3, max_iter=100, lam=1e-2):
    lo = np.array([LIM[i][0] for i in range(1, 6)])
    hi = np.array([LIM[i][1] for i in range(1, 6)])

    # желаемое направление оси инструмента (то же определение pitch, что в аналитике)
    phi = np.arctan2(y, x)
    d = np.array([np.sin(pitch) * np.cos(phi),
                  np.sin(pitch) * np.sin(phi),
                  np.cos(pitch)])
    target = np.array([x, y, z])

    def resid(q):
        p, a = fk_pose(q)
        return np.concatenate([p - target, w_ori * (a - d), [w_ori * (q[4] - roll)]])

    seeds = []
    if q0 is not None:
        seeds.append(np.clip(np.array(q0, float), lo, hi))
    seeds.append(np.array([phi, 0.5, -1.0, -0.5, roll]))   # запасные старты
    seeds.append(np.array([phi, 0.0, -0.5, -1.0, roll]))

    sols = []
    for q in seeds:
        q = np.clip(q, lo, hi)
        for _ in range(max_iter):
            r = resid(q)
            J = np.zeros((len(r), 5))
            for j in range(5):                      # конечные разности
                dq = np.zeros(5); dq[j] = 1e-6
                J[:, j] = (resid(q + dq) - r) / 1e-6
            step_q = -np.linalg.solve(J.T @ J + lam * np.eye(5), J.T @ r)
            step_q = np.clip(step_q, -0.3, 0.3)     # ограничение шага
            q = np.clip(q + step_q, lo, hi)         # лимиты суставов
            if np.linalg.norm(r) < 1e-7:
                break
        p, a = fk_pose(q)
        if (np.linalg.norm(p - target) < tol_pos
                and np.linalg.norm(a - d) < tol_ori
                and abs(q[4] - roll) < tol_ori):
            if not any(np.allclose(q, s, atol=1e-3) for s in sols):
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
        joints[i].setPosition(angs[i])
 
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

def createPoints(now, target, count):
    div = []
    for i in range(3):
            delta = (target[i] - now[i][3]) / count
            div.append(delta)
    res = []
    res.append((now[0][3], now[1][3], now[2][3])) 
    for i in range(1, count):            
        res.append((now[0][3] + div[0]* i, now[1][3] + div[1]* i, now[2][3] + div[2]* i)) 
    res.append(target)
    return res

def pick_closest(sols, current):
    cur = np.array(current, float)
    return min(sols, key=lambda q: np.linalg.norm(q - cur))

def manipulatorMove(need):
    angles = [sensors[i].getValue() for i in range(5)]
    M = fk(angles)
    
    points = createPoints(M, need, 7)
    for target in points:
        angles = [sensors[i].getValue() for i in range(5)]
        sols = ik_num(*target, pitch=np.pi/2, roll=np.pi/2, q0=angles)
        if not sols:
            continue
        q = pick_closest(sols, angles)
        if len(q) != 0:
            set_pose(q)
            while (result_eps(M, target) > NEED_EPS):
                angles = [sensors[i].getValue() for i in range(5)]
                M = fk(angles)
                step()
    
def transportationPose():
    target = (-0.25, -0.05, 0.4)
    manipulatorMove(target)
    
def takePlate(target):
    manipulatorMove(target)
    gripperState(1)
    
def placePlate(target):
    manipulatorMove((target[0], target[1] + 0.1, target[2]))
    manipulatorMove(target)
    gripperState(0)
    
    
def main():
    if not init_joints():
        return
    step()

    start = translation_field.getSFVec3f()
    
    gripperState(0)
    takePlate((0, -0.3, 0.4))
    transportationPose()
    placePlate((0, -0.3, 0.4))

    while True:
        step()

main()
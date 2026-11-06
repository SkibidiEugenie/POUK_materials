# -*- coding: utf-8 -*-
"""
ЛР1: генерация траектории-окружности и обработка телеметрии.

Использование:
  python lab1_traj_and_plots.py gen  [R_mm] [cx_mm] [cy_mm] [cz_mm] [N]
      -> circle_traj.h (в папку каркаса) + traj_ideal.csv
  python lab1_traj_and_plots.py log  COM5 [out.csv]
      -> пишет строки телеметрии 'T,ms,p1..p6' в CSV (нужен pyserial)
  python lab1_traj_and_plots.py plot log.csv
      -> plot_circle.png: реальная окружность (ПЗК по телеметрии) поверх заданной

Совместимо с Python 3.7. Кинематика и единицы — приложение А к ЛР1 (§А.3–А.5).
"""
import io
import math
import os
import sys

# ---------------- параметры стенда (проверьте по своей сборке) ---------------
L1, L2, L3, L4 = 0.103, 0.1045, 0.079, 0.169     # м
DEL = 0.00511327                                  # рад в отсчёте
HERE = os.path.dirname(os.path.abspath(__file__))
SKELETON_DIR = os.path.join(HERE, 'lab1_skeleton')

# значения по умолчанию для окружности
R_MM, CX_MM, CY_MM, CZ_MM, N_PTS = 30.0, 100.0, 0.0, 30.0, 20


def rad2ticks(a):
    return 512 + int(round(a / DEL))


def ticks2rad(t):
    return (t - 512) * DEL


def solve_ik(x, y, z):
    """ОЗК при вертикальном схвате. Вход: метры. Возврат: [a1..a5] (рад) или None."""
    k = math.sqrt(x * x + y * y)
    z_r = z + (L4 - L1)
    d = math.sqrt(k * k + z_r * z_r)
    if d < abs(L2 - L3) or d > (L2 + L3):
        return None
    c1 = (d * d + L2 * L2 - L3 * L3) / (2 * d * L2)
    c2 = (-d * d + L2 * L2 + L3 * L3) / (2 * L2 * L3)
    if abs(c1) > 1.0 or abs(c2) > 1.0:
        return None
    a1 = math.atan2(y, x)
    a2 = math.pi / 2 - (math.atan2(z_r, k) + math.acos(c1))
    a3 = math.pi - math.acos(c2)
    a4 = math.pi - (a2 + a3)
    a5 = a1
    return [a1, a2, a3, a4, a5]


def fk(a1, a2, a3):
    """ПЗК плоской части (углы от вертикали) + поворот основания. Возврат: x,y,z (м)."""
    k = L2 * math.sin(a2) + L3 * math.sin(a2 + a3)
    z = L1 + L2 * math.cos(a2) + L3 * math.cos(a2 + a3) - L4
    return k * math.cos(a1), k * math.sin(a1), z


# ------------------------------------------------------------------ gen ------
def cmd_gen(argv):
    r = float(argv[0]) if len(argv) > 0 else R_MM
    cx = float(argv[1]) if len(argv) > 1 else CX_MM
    cy = float(argv[2]) if len(argv) > 2 else CY_MM
    cz = float(argv[3]) if len(argv) > 3 else CZ_MM
    n = int(argv[4]) if len(argv) > 4 else N_PTS

    sagitta = r * (1 - math.cos(math.pi / n))
    print('R=%.1f mm, центр (%.1f, %.1f, %.1f) mm, N=%d, стрелка сегмента e=%.3f mm'
          % (r, cx, cy, cz, n, sagitta))

    cols = []          # список по точкам: [тики шести приводов]
    ideal = []
    for i in range(n):
        th = 2 * math.pi * i / n
        x = (cx + r * math.cos(th)) * 1e-3
        y = (cy + r * math.sin(th)) * 1e-3
        z = cz * 1e-3
        a = solve_ik(x, y, z)
        if a is None:
            print('!! точка %d (%.1f, %.1f) недостижима - измените центр/радиус' % (i, x * 1e3, y * 1e3))
            return 1
        ticks = [rad2ticks(v) for v in a] + [512]      # ID6 (схват) = 512
        bad = [j for j, t in enumerate(ticks) if not (0 <= t <= 1023)]
        if bad:
            print('!! точка %d: тики вне 0..1023 у суставов %s' % (i, bad))
            return 1
        cols.append(ticks)
        ideal.append((x, y, z))

    # circle_traj.h: массив [5 суставов][N точек] (схват не входит в обход)
    hpath = os.path.join(SKELETON_DIR, 'circle_traj.h')
    with io.open(hpath, 'w', encoding='utf-8') as f:
        f.write('// Автогенерация: lab1_traj_and_plots.py gen %s\n' % ' '.join(argv) if argv else
                '// Автогенерация: lab1_traj_and_plots.py gen (параметры по умолчанию)\n')
        f.write('// R=%.1f мм, центр (%.1f, %.1f, %.1f) мм, N=%d, стрелка e=%.3f мм\n'
                % (r, cx, cy, cz, n, sagitta))
        f.write('#pragma once\n#include <stdint.h>\n\n')
        f.write('const uint16_t TRAJ_N = %d;\n' % n)
        f.write('const int32_t CIRCLE_TRAJ[5][%d] = {\n' % n)
        for j in range(5):
            row = ', '.join('%4d' % cols[i][j] for i in range(n))
            f.write('  { %s }%s\n' % (row, ',' if j < 4 else ''))
        f.write('};\n')
    print('OK ->', hpath)

    cpath = os.path.join(HERE, 'traj_ideal.csv')
    with io.open(cpath, 'w', encoding='utf-8') as f:
        f.write('x_mm,y_mm,z_mm\n')
        for x, y, z in ideal:
            f.write('%.3f,%.3f,%.3f\n' % (x * 1e3, y * 1e3, z * 1e3))
    print('OK ->', cpath)
    return 0


# ------------------------------------------------------------------ log ------
def cmd_log(argv):
    if not argv:
        print('usage: log COM5 [out.csv]')
        return 1
    port, out = argv[0], (argv[1] if len(argv) > 1 else 'log.csv')
    try:
        import serial
    except ImportError:
        print('Нужен pyserial: pip install pyserial')
        return 1
    ser = serial.Serial(port, 57600, timeout=1)
    print('Пишу %s -> %s. Запустите на стенде команду C. Ctrl+C - стоп.' % (port, out))
    n = 0
    with io.open(out, 'w', encoding='utf-8') as f:
        f.write('ms,p1,p2,p3,p4,p5,p6\n')
        try:
            while True:
                line = ser.readline().decode('utf-8', 'ignore').strip()
                if line.startswith('T,'):
                    f.write(line[2:] + '\n')
                    n += 1
                elif line:
                    print(' ', line)
        except KeyboardInterrupt:
            pass
    print('Строк телеметрии:', n)
    return 0


# ------------------------------------------------------------------ plot -----
def cmd_plot(argv):
    if not argv:
        print('usage: plot log.csv')
        return 1
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    xs, ys = [], []
    with io.open(argv[0], 'r', encoding='utf-8') as f:
        header = f.readline()
        for line in f:
            parts = line.strip().split(',')
            if len(parts) < 7:
                continue
            t = [int(p) for p in parts[1:7]]
            a1, a2, a3 = ticks2rad(t[0]), ticks2rad(t[1]), ticks2rad(t[2])
            x, y, _ = fk(a1, a2, a3)
            xs.append(x * 1e3)
            ys.append(y * 1e3)
    if not xs:
        print('В логе нет телеметрии')
        return 1

    fig, ax = plt.subplots(figsize=(6, 6))
    ipath = os.path.join(HERE, 'traj_ideal.csv')
    if os.path.exists(ipath):
        ix, iy = [], []
        with io.open(ipath, 'r', encoding='utf-8') as f:
            f.readline()
            for line in f:
                p = line.strip().split(',')
                ix.append(float(p[0])); iy.append(float(p[1]))
        ix.append(ix[0]); iy.append(iy[0])
        ax.plot(ix, iy, '--', color='#2C7A5A', lw=1.6, label=u'заданная')
    ax.plot(xs, ys, '-', color='#B85042', lw=1.4, label=u'реальная (ПЗК по телеметрии)')
    ax.set_xlabel('x, мм'); ax.set_ylabel('y, мм')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.4); ax.legend()
    ax.set_title(u'ЛР1: заданная и реальная траектории схвата')
    out = os.path.join(HERE, 'plot_circle.png')
    fig.savefig(out, dpi=150, bbox_inches='tight')
    print('OK ->', out)
    return 0


if __name__ == '__main__':
    cmds = {'gen': cmd_gen, 'log': cmd_log, 'plot': cmd_plot}
    if len(sys.argv) < 2 or sys.argv[1] not in cmds:
        print(__doc__)
        sys.exit(1)
    sys.exit(cmds[sys.argv[1]](sys.argv[2:]))

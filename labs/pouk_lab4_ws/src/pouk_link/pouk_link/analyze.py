# -*- coding: utf-8 -*-
"""Обработка журналов стенда для отчётов ЛР2–ЛР4.

    python3 -m pouk_link.analyze latency  PREFIX [--out DIR]
        период телеметрии и время оборота команд: p50, p99, p99,9, максимум,
        потери; гистограммы с логарифмической осью (пособие, §2.1.3, §2.3.5)
    python3 -m pouk_link.analyze tracking PREFIX [--r 30 --cx 100 --cy 0 --cz 30]
        траектория схвата по телеметрии (ПЗК) поверх заданной окружности
    python3 -m pouk_link.analyze watchdog PREFIX
        срабатывание сторожевого таймера: задержка и перемещение после последней команды

Таблицы печатаются в формате Markdown — их можно вставлять в отчёт.
Графики требуют matplotlib (на ПК); расчёты работают и без него (на одноплатнике).
"""
import argparse
import csv
import io
import math
import os
import sys

from . import kinematics as K
from . import protocol as P


def percentile(sorted_vals, p):
    """Процентиль с линейной интерполяцией; sorted_vals отсортирован по возрастанию."""
    if not sorted_vals:
        return float('nan')
    if len(sorted_vals) == 1:
        return sorted_vals[0]
    pos = (len(sorted_vals) - 1) * p / 100.0
    lo = int(math.floor(pos))
    hi = min(lo + 1, len(sorted_vals) - 1)
    return sorted_vals[lo] + (sorted_vals[hi] - sorted_vals[lo]) * (pos - lo)


def summary(vals):
    s = sorted(vals)
    return dict(n=len(s), mean=(sum(s) / len(s)) if s else float('nan'),
                p50=percentile(s, 50), p90=percentile(s, 90), p99=percentile(s, 99),
                p999=percentile(s, 99.9), max=s[-1] if s else float('nan'))


def read_telem(prefix):
    rows = []
    with io.open(prefix + '_telem.csv', encoding='utf-8') as f:
        for r in csv.DictReader(f):
            rows.append(dict(t_host=float(r['t_host']), stamp=float(r['stamp']),
                             session=int(r['session']), seq=int(r['seq']), t_mcu=int(r['t_mcu_ms']),
                             q=[float(r['q%d' % i]) for i in range(1, 7)],
                             state=int(r['state']), flags=int(r['flags']), last_cmd=int(r['last_cmd'])))
    return rows


def read_cmd(prefix):
    rows = []
    path = prefix + '_cmd.csv'
    if not os.path.exists(path):
        return rows
    with io.open(path, encoding='utf-8') as f:
        for r in csv.DictReader(f):
            rows.append(dict(cmd_id=int(r['cmd_id']), t_send=float(r['t_send']),
                             t_ack=float(r['t_ack']) if r['t_ack'] else None,
                             attempts=int(r['attempts']),
                             result=int(r['result']) if r['result'] else None,
                             reason=int(r['reason']) if r['reason'] else None))
    return rows


def _md_row(name, s, unit_scale=1000.0):
    return '| %s | %d | %.2f | %.2f | %.2f | %.2f | %.2f |' % (
        name, s['n'], s['p50'] * unit_scale, s['p90'] * unit_scale, s['p99'] * unit_scale,
        s['p999'] * unit_scale, s['max'] * unit_scale)


def _hist(vals_ms, title, path, xlabel):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('(matplotlib не установлен — график %s пропущен)' % os.path.basename(path))
        return
    s = sorted(vals_ms)
    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    ax.hist(vals_ms, bins=120, color='#1F3A6E', alpha=0.85)
    ax.set_yscale('log')
    for p, c in ((50, '#2C7A5A'), (99, '#D98A1A'), (99.9, '#B85042')):
        v = percentile(s, p)
        ax.axvline(v, color=c, lw=1.4, ls='--', label='p%s = %.2f мс' % (p, v))
    ax.axvline(s[-1], color='black', lw=1.0, ls=':', label='max = %.2f мс' % s[-1])
    ax.set_xlabel(xlabel)
    ax.set_ylabel('число событий (лог.)')
    ax.set_title(title)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print('график ->', path)


def cmd_latency(a):
    tel = read_telem(a.prefix)
    cmds = read_cmd(a.prefix)
    out = a.out or os.path.dirname(os.path.abspath(a.prefix))
    periods, lost, sessions = [], 0, set()
    for prev, cur in zip(tel, tel[1:]):
        if cur['session'] != prev['session']:
            sessions.add(cur['session'])
            continue
        gap = P.seq_distance(cur['seq'], prev['seq'])
        lost += gap - 1
        if gap == 1:
            periods.append(cur['t_host'] - prev['t_host'])
    rtt = [c['t_ack'] - c['t_send'] for c in cmds if c['t_ack'] is not None]
    retried = sum(1 for c in cmds if c['attempts'] > 1)
    cmd_lost = sum(1 for c in cmds if c['t_ack'] is None)
    dur = (tel[-1]['t_host'] - tel[0]['t_host']) if len(tel) > 1 else 0.0
    print('Длительность записи: %.1f мин; кадров телеметрии: %d; потеряно: %d (%.4f %%)'
          % (dur / 60, len(tel), lost, 100.0 * lost / max(1, len(tel) + lost)))
    print('Команд: %d; с повтором: %d; без подтверждения: %d' % (len(cmds), retried, cmd_lost))
    print()
    print('| Величина, мс | n | p50 | p90 | p99 | p99,9 | max |')
    print('|---|---|---|---|---|---|---|')
    if periods:
        print(_md_row('Период приёма телеметрии', summary(periods)))
    if rtt:
        print(_md_row('Оборот команда — подтверждение', summary(rtt)))
    print()
    if periods:
        _hist([v * 1000 for v in periods], 'Период приёма телеметрии',
              os.path.join(out, 'hist_telem_period.png'), 'мс')
    if rtt:
        _hist([v * 1000 for v in rtt], 'Время оборота «команда — подтверждение»',
              os.path.join(out, 'hist_cmd_rtt.png'), 'мс')
    return 0


def cmd_tracking(a):
    tel = read_telem(a.prefix)
    pts = [K.fk(r['q']) for r in tel if r['state'] == P.ST_RUN]
    if not pts:
        print('В журнале нет телеметрии в режиме RUN')
        return 1
    cx, cy, r0 = a.cx / 1e3, a.cy / 1e3, a.r / 1e3
    radial = [abs(math.hypot(x - cx, y - cy) - r0) * 1e3 for x, y, _ in pts]
    near = [e for e, (x, y, _) in zip(radial, pts) if math.hypot(x - cx, y - cy) > 0.3 * r0]
    s = summary(near) if near else None
    if s:
        print('Радиальное отклонение от окружности, мм: среднее %.2f, p99 %.2f, максимум %.2f (n=%d)'
              % (s['mean'], s['p99'], s['max'], s['n']))
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('(matplotlib не установлен — график пропущен)')
        return 0
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    th = [2 * math.pi * i / 360 for i in range(361)]
    ax.plot([a.cx + a.r * math.cos(t) for t in th], [a.cy + a.r * math.sin(t) for t in th], '--',
            color='#2C7A5A', lw=1.6, label='заданная')
    ax.plot([p[0] * 1e3 for p in pts], [p[1] * 1e3 for p in pts], '-', color='#B85042', lw=1.0,
            label='по телеметрии (ПЗК)')
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.4)
    ax.set_xlabel('x, мм')
    ax.set_ylabel('y, мм')
    ax.legend()
    path = os.path.join(a.out or os.path.dirname(os.path.abspath(a.prefix)), 'tracking.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print('график ->', path)
    return 0


def cmd_watchdog(a):
    tel = read_telem(a.prefix)
    cmds = read_cmd(a.prefix)
    trips = [i for i in range(1, len(tel)) if tel[i]['state'] == P.ST_WD_STOP
             and tel[i - 1]['state'] != P.ST_WD_STOP]
    if not trips:
        print('Срабатываний сторожевого таймера в журнале нет')
        return 1
    for i in trips:
        t_trip = tel[i]['t_host']
        sent = [c['t_send'] for c in cmds if c['t_send'] < t_trip]
        t_last = max(sent) if sent else float('nan')
        p_last = min(tel[:i + 1], key=lambda r: abs(r['t_host'] - t_last))
        after = [r for r in tel[i:] if r['t_host'] - t_trip <= 2.0]
        p_end = after[-1]
        x0, y0, z0 = K.fk(p_last['q'])
        x1, y1, z1 = K.fk(p_end['q'])
        print('Срабатывание: через %.1f мс после последней отправленной команды; '
              'перемещение схвата после неё %.1f мм' % ((t_trip - t_last) * 1000,
                                                       math.dist((x0, y0, z0), (x1, y1, z1)) * 1000
                                                       if hasattr(math, 'dist') else
                                                       math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2
                                                                 + (z1 - z0) ** 2) * 1000))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog='pouk_link.analyze', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd')
    for name in ('latency', 'tracking', 'watchdog'):
        p = sub.add_parser(name)
        p.add_argument('prefix')
        p.add_argument('--out', default=None)
        if name == 'tracking':
            p.add_argument('--r', type=float, default=30.0)
            p.add_argument('--cx', type=float, default=100.0)
            p.add_argument('--cy', type=float, default=0.0)
            p.add_argument('--cz', type=float, default=30.0)
    a = ap.parse_args(argv)
    if not a.cmd:
        ap.print_help()
        return 1
    return {'latency': cmd_latency, 'tracking': cmd_tracking, 'watchdog': cmd_watchdog}[a.cmd](a)


if __name__ == '__main__':
    sys.exit(main())

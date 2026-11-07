# -*- coding: utf-8 -*-
"""Канал «одноплатник ↔ микроконтроллер» по протоколу стенда v2.

StandLink владеет последовательным портом и двумя потоками:
  * поток приёма разбирает кадры автоматом Receiver и раскладывает пакеты:
    телеметрия -> регистр «последнее значение», подтверждения -> CommandTracker;
  * поток передачи раз в миллисекунду выполняет повторы по тайм-ауту и, если
    включён потоковый режим, выдаёт уставки с постоянным периодом (сторожевой
    таймер микроконтроллера требует непрерывного потока команд).

Модуль не зависит от ROS: тот же код работает в CLI ЛР2 и внутри узла-моста ЛР3–ЛР4.
Порт задаётся строкой: '/dev/ttyS1' (стенд), '/dev/ttyACM0' (OpenCM по USB),
'socket://127.0.0.1:5760' (виртуальный стенд pouk_link.sim_mcu).
"""
import collections
import io
import random
import threading
import time
from collections import namedtuple

from . import protocol as P

TelemetrySample = namedtuple('TelemetrySample',
                             't_host stamp session seq t_ms q state flags last_cmd')
CmdResult = namedtuple('CmdResult', 'cmd_id ok result reason attempts rtt')


# ======================================================================
class ClockMapper(object):
    """Перевод времени МК (t_ms) в шкалу одноплатника.

    Смещение шкал оценивается минимумом (t_приёма − t_МК) на скользящем окне:
    минимальная задержка доставки почти постоянна, а переменная часть
    (планирование ОС, очереди) в минимум не попадает. Метка времени образца
    получается как t_МК + смещение — момент измерения, а не момент приёма
    (пособие, §3.5.2). Дрейф кварца отслеживается сдвигом окна.
    """

    def __init__(self, window_s=10.0):
        self.window_s = window_s
        self._q = collections.deque()      # (t_host, offset), offset неубывающий по окну
        self._session = None
        self._last_t_ms = None

    def update(self, t_host, t_ms, session):
        if session != self._session or (self._last_t_ms is not None and t_ms < self._last_t_ms):
            self._q.clear()                # перезапуск МК: шкала сброшена
            self._session = session
        self._last_t_ms = t_ms
        off = t_host - t_ms / 1000.0
        while self._q and self._q[-1][1] >= off:
            self._q.pop()
        self._q.append((t_host, off))
        while self._q and t_host - self._q[0][0] > self.window_s:
            self._q.popleft()
        return t_ms / 1000.0 + self._q[0][1]


# ======================================================================
class CommandTracker(object):
    """Учёт команд: ожидание подтверждения, однократный повтор, статистика.

    Команда отправлена -> ждём ACK не дольше ack_timeout -> если нет, повторяем
    ТОТ ЖЕ кадр с тем же cmd_id (команды абсолютные, повтор безвреден,
    пособие §2.6.4) -> после max_attempts попыток команда считается потерянной.
    """

    Pending = namedtuple('Pending', 'cmd_id frame t_first deadline attempts event')

    def __init__(self, ack_timeout=0.008, max_attempts=2, rtt_keep=200000):
        self.ack_timeout = ack_timeout
        self.max_attempts = max_attempts
        self.lock = threading.Lock()
        self.pending = {}                  # cmd_id -> Pending
        self.results = {}                  # cmd_id -> CmdResult (для ожидающих)
        self.rtt = collections.deque(maxlen=rtt_keep)
        self.counters = collections.Counter()
        self.log = None                    # файл журнала команд или None

    def register(self, cmd_id, frame, now, event=None):
        with self.lock:
            self.pending[cmd_id] = self.Pending(cmd_id, frame, now, now + self.ack_timeout, 1, event)
            self.counters['cmd_sent'] += 1

    def on_ack(self, ack, now):
        """Обработать подтверждение. Возвращает CmdResult или None (чужое/запоздавшее)."""
        with self.lock:
            # >>> STUDENT ЛР2-шаг5: найти ожидающую команду по ack.cmd_id и завершить её:
            # >>> время оборота rtt = now − t_first (от ПЕРВОЙ отправки), счётчики
            # >>> Подтверждение команды, которой нет в таблице, — счётчик ack_unexpected.
            pending = self.pending.pop(ack.cmd_id, None)
            if pending is None:
                self.counters['ack_unexpected'] += 1
                return None

            rtt = now - pending.t_first
            ok = ack.result == P.ACCEPTED
            if ok:
                self.counters['cmd_acked'] += 1
            else:
                self.counters['cmd_rejected'] += 1

            result = CmdResult(
                cmd_id=ack.cmd_id,
                ok=ok,
                result=ack.result,
                reason=ack.reason,
                attempts=pending.attempts,
                rtt=rtt,
            )
            self.results[ack.cmd_id] = result
            self.rtt.append(rtt)

            if self.log is not None:
                self.log.write(
                    'ACK,%d,%d,%d,%d,%.6f\n'
                    % (ack.cmd_id, ack.result, ack.reason,
                       pending.attempts, rtt)
                )

            if pending.event is not None:
                pending.event.set()

            return result
            # <<< STUDENT

    def due_retries(self, now):
        """Кадры, которые пора повторить. Команды без попыток — в потерянные."""
        out = []
        with self.lock:
            # >>> STUDENT ЛР2-шаг5: для каждой ожидающей команды с истёкшим deadline:
            # >>> если попыток меньше max_attempts — вернуть кадр на повтор и обновить
            # >>> deadline и attempts (счётчик cmd_retries); иначе удалить из таблицы,
            # >>> увеличить cmd_lost, записать в журнал и разбудить ожидающий поток.
            for cmd_id in list(self.pending.keys()):
                pending = self.pending[cmd_id]
                if now < pending.deadline:
                    continue

                if pending.attempts < self.max_attempts:
                    self.counters['cmd_retries'] += 1
                    self.pending[cmd_id] = pending._replace(
                        attempts=pending.attempts + 1,
                        deadline=now + self.ack_timeout,
                    )
                    out.append(pending.frame)

                    if self.log is not None:
                        self.log.write(
                            'RETRY,%d,%d\n'
                            % (cmd_id, pending.attempts + 1)
                        )
                    continue

                del self.pending[cmd_id]
                self.counters['cmd_lost'] += 1

                result = CmdResult(
                    cmd_id=cmd_id,
                    ok=False,
                    result=None,
                    reason=None,
                    attempts=pending.attempts,
                    rtt=None,
                )
                self.results[cmd_id] = result

                if self.log is not None:
                    self.log.write('LOST,%d,%d\n' % (cmd_id, pending.attempts))

                if pending.event is not None:
                    pending.event.set()
            # <<< STUDENT
        return out

    def take_result(self, cmd_id):
        with self.lock:
            return self.results.pop(cmd_id, None)

    def next_deadline(self):
        """Ближайший срок ожидания подтверждения или None, если ждать нечего."""
        with self.lock:
            return min((p.deadline for p in self.pending.values()), default=None)


# ======================================================================
class FaultInjector(object):
    """Искусственные помехи на передаче: проверка диагностируемости канала.

    drop   — вероятность потерять кадр целиком;
    ber    — вероятность инверсии каждого бита;
    trunc  — вероятность обрезать кадр (без разделителя: проверка тайм-аута и ресинхронизации).
    """

    def __init__(self, drop=0.0, ber=0.0, trunc=0.0, seed=1):
        self.drop, self.ber, self.trunc = drop, ber, trunc
        self.rng = random.Random(seed)
        self.counters = collections.Counter()

    def active(self):
        return self.drop > 0 or self.ber > 0 or self.trunc > 0

    def apply(self, frame):
        if self.drop > 0 and self.rng.random() < self.drop:
            self.counters['inj_drop'] += 1
            return b''
        out = bytearray(frame)
        if self.ber > 0:
            flipped = False
            for i in range(len(out)):
                for bit in range(8):
                    if self.rng.random() < self.ber:
                        out[i] ^= (1 << bit)
                        flipped = True
            if flipped:
                self.counters['inj_corrupt'] += 1
        if self.trunc > 0 and self.rng.random() < self.trunc and len(out) > 2:
            out = out[:self.rng.randrange(1, len(out) - 1)]
            self.counters['inj_trunc'] += 1
        return bytes(out)


# ======================================================================
class StandLink(object):
    def __init__(self, port, baud=P.BAUD, ack_timeout=0.008, max_attempts=2,
                 interbyte_timeout=0.05, fault=None, session=None):
        self.port = port
        self.baud = baud
        self.fault = fault or FaultInjector()
        self.rx = P.Receiver(interbyte_timeout=interbyte_timeout)
        self.tracker = CommandTracker(ack_timeout, max_attempts)
        self.clock = ClockMapper()
        self.session = session if session is not None else random.randrange(1, 256)
        self.seq = {}                      # номер пакета ведётся по каждому типу
        self.next_cmd_id = random.randrange(1, 1000)
        self.counters = collections.Counter()
        self.ser = None
        self._wlock = threading.Lock()
        self._state_lock = threading.Lock()
        self._latest = None
        self._latest_diag = None
        self._mcu_session = None
        self._last_seq = None
        self._callbacks = []
        self._telem_log = None
        self._running = False
        self._threads = []
        self._stream_on = False
        self.stream_hz = 50.0
        self._stream_period = 0.02
        self._stream_next = 0.0
        self._target = None
        self._target_vmax = 0.0

    # ------------------------------------------------------------ жизненный цикл
    def open(self):
        import serial                                   # pyserial
        self.ser = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.01,
                                         write_timeout=0.5)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        return self

    def start(self):
        if self.ser is None:
            self.open()
        self._running = True
        for fn, name in ((self._rx_loop, 'pouk-rx'), (self._tx_loop, 'pouk-tx')):
            th = threading.Thread(target=fn, name=name)
            th.daemon = True
            th.start()
            self._threads.append(th)
        return self

    def close(self):
        self._running = False
        for th in self._threads:
            th.join(timeout=1.0)
        self._threads = []
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        for f in (self._telem_log, self.tracker.log):
            if f is not None:
                f.close()
        self._telem_log = None
        self.tracker.log = None

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------ журналы
    def log_to(self, prefix):
        """Писать телеметрию в <prefix>_telem.csv, команды — в <prefix>_cmd.csv."""
        self._telem_log = io.open(prefix + '_telem.csv', 'w', encoding='utf-8', buffering=1 << 16)
        self._telem_log.write('t_host,stamp,session,seq,t_mcu_ms,q1,q2,q3,q4,q5,q6,state,flags,last_cmd\n')
        self.tracker.log = io.open(prefix + '_cmd.csv', 'w', encoding='utf-8', buffering=1 << 16)
        self.tracker.log.write('cmd_id,t_send,t_ack,attempts,result,reason\n')

    def on_telemetry(self, cb):
        self._callbacks.append(cb)

    # ------------------------------------------------------------ приём
    def _rx_loop(self):
        while self._running:
            try:
                n = self.ser.in_waiting
                data = self.ser.read(n if n > 1 else 1)
            except Exception:
                self.counters['port_error'] += 1
                time.sleep(0.05)
                continue
            now = time.monotonic()
            results = self.rx.feed(data, now) if data else self.rx.poll(now)
            for r in results:
                if isinstance(r, str):
                    self.counters[r] += 1
                else:
                    self._dispatch(r, now)

    def _dispatch(self, pkt, now):
        self.counters['rx_packets'] += 1
        if pkt.type == P.T_TELEM:
            self._on_telem(pkt, P.unpack(pkt), now)
        elif pkt.type == P.T_ACK:
            self.tracker.on_ack(P.unpack(pkt), now)
        elif pkt.type == P.T_DIAG:
            with self._state_lock:
                self._latest_diag = P.unpack(pkt)
        else:
            self.counters['type_err'] += 1

    def _on_telem(self, pkt, t, now):
        if pkt.session != self._mcu_session:
            if self._mcu_session is not None:
                self.counters['mcu_restarts'] += 1
            self._mcu_session = pkt.session
            self._last_seq = None
        if self._last_seq is not None:
            gap = P.seq_distance(pkt.seq, self._last_seq)
            if gap == 0 or gap > 32768:
                self.counters['telem_dup_or_old'] += 1
                return
            self.counters['telem_lost'] += gap - 1
        self._last_seq = pkt.seq
        stamp = self.clock.update(now, t.t_ms, pkt.session)
        s = TelemetrySample(now, stamp, pkt.session, pkt.seq, t.t_ms, t.q, t.state, t.flags, t.last_cmd)
        with self._state_lock:
            self._latest = s
        self.counters['telem_ok'] += 1
        if self._telem_log is not None:
            self._telem_log.write('%.6f,%.6f,%d,%d,%d,%s,%d,%d,%d\n' % (
                now, stamp, pkt.session, pkt.seq, t.t_ms, ','.join('%.4f' % a for a in t.q),
                t.state, t.flags, t.last_cmd))
        for cb in self._callbacks:
            cb(s)

    # ------------------------------------------------------------ передача
    def _write(self, frame):
        frame = self.fault.apply(frame) if self.fault.active() else frame
        if not frame:
            return
        with self._wlock:
            try:
                self.ser.write(frame)
                self.counters['tx_bytes'] += len(frame)
            except Exception:
                self.counters['port_error'] += 1

    def _make_frame(self, ptype, payload):
        with self._wlock:
            seq = self.seq.get(ptype, 0)
            self.seq[ptype] = (seq + 1) & 0xFFFF
        return P.build_frame(ptype, self.session, seq, payload)

    def _alloc_cmd_id(self):
        with self._wlock:
            cid = self.next_cmd_id
            self.next_cmd_id = (self.next_cmd_id + 1) & 0xFFFF or 1
        return cid

    def _tx_loop(self):
        while self._running:
            now = time.monotonic()
            for frame in self.tracker.due_retries(now):
                self._write(frame)
            if self._stream_on and now >= self._stream_next:
                with self._state_lock:
                    target, vmax = self._target, self._target_vmax
                if target is not None:
                    cid = self._alloc_cmd_id()
                    frame = self._make_frame(P.T_CMD_JOINTS, P.pack_cmd_joints(cid, target, vmax))
                    self.tracker.register(cid, frame, time.monotonic())
                    self._write(frame)
                # следующий момент считается от расписания, а не от «сейчас»:
                # иначе ошибки накапливаются и период «уплывает»
                self._stream_next += self._stream_period
                if self._stream_next < now:
                    self._stream_next = now + self._stream_period
            # спать до ближайшего события (очередная уставка или срок подтверждения), но не дольше 5 мс:
            # поток, просыпающийся каждую миллисекунду, в Python постоянно борется за GIL с остальными
            wake = now + 0.005
            if self._stream_on:
                wake = min(wake, self._stream_next)
            deadline = self.tracker.next_deadline()
            if deadline is not None:
                wake = min(wake, deadline)
            delay = wake - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    # ------------------------------------------------------------ команды
    def _command(self, ptype, payload_fn, wait=True):
        cid = self._alloc_cmd_id()
        frame = self._make_frame(ptype, payload_fn(cid))
        ev = threading.Event() if wait else None
        self.tracker.register(cid, frame, time.monotonic(), ev)
        self._write(frame)
        if not wait:
            return CmdResult(cid, None, None, None, 1, None)
        ev.wait(self.tracker.ack_timeout * self.tracker.max_attempts + 0.2)
        res = self.tracker.take_result(cid)
        return res if res is not None else CmdResult(cid, False, None, None, 0, None)

    def set_mode(self, mode, wait=True):
        m = P.MODE_BY_NAME[mode] if isinstance(mode, str) else mode
        return self._command(P.T_CMD_MODE, lambda cid: P.pack_cmd_mode(cid, m), wait)

    def send_joints(self, q, vmax=0.0, wait=True):
        return self._command(P.T_CMD_JOINTS, lambda cid: P.pack_cmd_joints(cid, q, vmax), wait)

    def stream_start(self, rate_hz=None):
        self._stream_period = 1.0 / (rate_hz or self.stream_hz)
        self._stream_next = time.monotonic()
        self._stream_on = True

    def stream_stop(self):
        self._stream_on = False

    @property
    def streaming(self):
        return self._stream_on

    def set_target(self, q, vmax=0.0):
        with self._state_lock:
            self._target = list(q)
            self._target_vmax = vmax

    # ------------------------------------------------------------ состояние
    def latest(self):
        with self._state_lock:
            return self._latest

    def latest_diag(self):
        with self._state_lock:
            return self._latest_diag

    def wait_telemetry(self, timeout=2.0):
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            s = self.latest()
            if s is not None:
                return s
            time.sleep(0.01)
        return None

    def wait_reached(self, q_target, eps=0.08, timeout=5.0, joints=range(5)):
        """Ждать, пока все указанные суставы окажутся в пределах eps (рад). True/False."""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            s = self.latest()
            if s is not None and all(abs(s.q[j] - q_target[j]) <= eps for j in joints):
                return True
            time.sleep(0.005)
        return False

    def stats(self):
        """Счётчики канала (Counter: отсутствующий ключ читается как 0)."""
        out = collections.Counter(self.counters)
        out.update(self.tracker.counters)
        out.update(self.fault.counters)
        return out

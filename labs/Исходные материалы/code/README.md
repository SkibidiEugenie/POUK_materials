# Код лабораторных работ ПОУК

Одна система на все четыре работы: каждая следующая работа использует код предыдущей.

```
code/
├── lab1/                        ЛР1: каркас прошивки OpenCM, генерация окружности, графики
├── firmware/
│   ├── pouk_mcu/                ЛР2: прошивка стенда с протоколом v2 (Arduino IDE, OpenCM9.04)
│   └── host_test/               ЛР2: тесты логики прошивки на компьютере (g++, без платы)
└── pouk_ws/src/                 рабочее пространство ROS 2 (colcon)
    ├── pouk_link/               ЛР2: протокол v2, канал, кинематика, виртуальный стенд, CLI, анализ журналов
    ├── pouk_interfaces/         ЛР3: сервисы координатора и зонда часов
    ├── pouk_description/        ЛР3: URDF стенда (совпадает с кинематикой приложения А)
    ├── pouk_bridge/             ЛР3–ЛР4-А: мост ROS 2 ↔ стенд, исполнитель FollowJointTrajectory
    ├── pouk_coordinator/        ЛР3: автомат состояний IDLE/HOMING/MOVE_POINT/CIRCLE/FAULT
    ├── pouk_moveit_config/      ЛР4-А: конфигурация MoveIt 2
    └── pouk_bringup/            ЛР3–ЛР4: описания запуска, профили, самопроверки, зонд часов, проверка TF
```

`pouk_link` не зависит от ROS: протокол, приложение одноплатника (ЛР2) и виртуальный стенд работают на чистом Python 3.10+ с `pyserial`. Остальные пакеты — ROS 2 Humble или Jazzy.

## Что пишет бригада

В коде размечены места для реализации:

```python
# >>> STUDENT ЛР2-шаг4: автомат приёмника. Реализуйте переходы по таблице ...
raise NotImplementedError('ЛР2, шаг 4: автомат приёмника (Receiver.feed)')
# <<< STUDENT
```

Замените заглушку своей реализацией между маркерами (маркеры можно оставить). Остальной код менять можно, но тогда объясните изменения в отчёте.

| Шаг | Файл | Что реализовать | Тесты, которые должны пройти |
|---|---|---|---|
| ЛР2-шаг4 | `pouk_link/pouk_link/protocol.py` | автомат приёмника кадров `Receiver.feed` | `pouk_link/test/test_receiver.py`, `test_protocol.py` |
| ЛР2-шаг5 | `pouk_link/pouk_link/link.py` | подтверждения и повторы `CommandTracker.on_ack`, `due_retries` | `test_tracker_kinematics.py`, затем `test_link_sim.py` |
| ЛР2-шаг6 | `firmware/pouk_mcu/mcu_logic.cpp` | дубликаты и устаревшие команды; пределы суставов до записи | `firmware/host_test` |
| ЛР2-шаг7 | `firmware/pouk_mcu/mcu_logic.cpp` | сторожевой таймер | `firmware/host_test` |
| ЛР3-шаг2 | `pouk_bridge/pouk_bridge/joint_map.py` | выборка суставов по именам | `pouk_bridge/test/test_joint_map.py` |
| ЛР3-шаг3 | `pouk_bridge/pouk_bridge/bridge_node.py` | профили QoS моста | `ros2 run pouk_bringup check_bridge --lab3` |
| ЛР3-шаг5 | `pouk_coordinator/pouk_coordinator/fsm.py` | таблица переходов автомата | `pouk_coordinator/test/test_fsm.py`, `check_coordinator` |
| ЛР4А-шаг1 | `pouk_moveit_config/config/kinematics.yaml`, `joint_limits.yaml` | режим ОЗК, пределы скорости и ускорения | `pouk_bringup/test/test_launch_files.py` |
| ЛР4А-шаг2 | `pouk_moveit_config/config/moveit_controllers.yaml` | описание исполнителя траектории | `test_launch_files.py` |
| ЛР4А-шаг3 | `pouk_bridge/pouk_bridge/trajectory.py` | интерполяция по содержимому сообщения | `test_trajectory.py` |
| ЛР4А-шаг4 | `pouk_bridge/pouk_bridge/executor.py` | проверка начала траектории, допуски и коды отказа | `test_executor.py` |
| ЛР4А-шаг5 | `pouk_bridge/pouk_bridge/bridge_node.py` | удержание при отказе, статус цели | `check_bridge`, `check_bridge --stuck`, `check_moveit` |

ЛР1 выполняется в `lab1/` по своей инструкции: там заготовки помечены `TODO шаг N`.

## Быстрый старт без стенда

**Только Python (ЛР2), любая ОС:**

```bash
cd code/pouk_ws/src/pouk_link
python3 -m pip install pyserial pytest matplotlib
python3 -m pytest -q test                                     # тесты протокола и канала
python3 -m pouk_link.sim_mcu                                  # терминал 1: виртуальный стенд на 127.0.0.1:5760
python3 -m pouk_link.cli --port socket://127.0.0.1:5760 home  # терминал 2: приложение одноплатника
```

**Тесты логики прошивки (ЛР2), Linux/WSL/macOS с g++:**

```bash
cd code/firmware/host_test && ./build_and_run.sh
```

**ROS 2 (ЛР3–ЛР4), Ubuntu 22.04 + Humble или 24.04 + Jazzy:**

```bash
sudo apt install python3-serial ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-robot-state-publisher \
     ros-$ROS_DISTRO-joint-state-publisher-gui ros-$ROS_DISTRO-control-msgs ros-$ROS_DISTRO-moveit
cd code/pouk_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --event-handlers console_direct- && colcon test-result --verbose

ros2 launch pouk_bringup sim_stand.launch.py        # терминал 1: модель стенда + мост
ros2 run pouk_bringup check_bridge --lab3            # терминал 2: самопроверка моста
```

## Программы самопроверки

| Команда | Что проверяет | Что запустить заранее |
|---|---|---|
| `ros2 run pouk_bringup check_bridge --lab3` | мост ЛР3: режимы, имена суставов, QoS, метки времени, пределы | `sim_stand.launch.py` |
| `ros2 run pouk_bringup check_bridge` | то же + контракт исполнителя траектории ЛР4-А: интерполяция, коды отказов, арбитраж, вытеснение, отмена | `sim_stand.launch.py` |
| `ros2 run pouk_bringup check_bridge --stuck` | отказ по допуску пути при заклинивании сустава | `sim_stand.launch.py stuck:=3@12` |
| `ros2 run pouk_bringup check_coordinator` | сценарии и отказы координатора | `lab3.launch.py sim:=true` |
| `ros2 run pouk_bringup check_moveit --variant N` | контур MoveIt → мост → стенд, обход препятствия своего варианта | `lab4a.launch.py sim:=true` |
| `ros2 run pouk_bringup clock_probe server` / `client` | смещение часов двух машин (схема NTP) | на второй машине — `server` |
| `ros2 run pouk_bringup tf_audit` | дерево координат: связность, единственный родитель, частоты, метки времени | работающая система |

Самопроверки подходят и для реального стенда, но выполняют движения: запускайте их при свободной рабочей зоне и с рукой на выключателе питания приводов.

## Если ROS 2 работает в WSL2

В WSL2 обнаружение узлов DDS по UDP часто не работает: топики не видны между терминалами, `ros2 topic echo` молчит. Для работы **в пределах одного компьютера** включите транспорт через разделяемую память во всех терминалах:

```bash
export FASTDDS_BUILTIN_TRANSPORTS=SHM
ros2 daemon stop
```

Многомашинная часть ЛР3 (ПК ↔ NanoPi) из WSL2 без дополнительной настройки сети не работает: используйте Ubuntu на компьютере или виртуальную машину с сетевым мостом.

# PCB-assembly-SCARA-

In this repository you will find all the code for our Robotics and Digital Control project, which consists in building an RRP SCARA that places SMD components like resistors, LEDs & buttons on a PCB.

**Authors:** Sebastián Escobar Restrepo, Marzuq Gallego, Santiago Ortiz, Laura Gutiérrez
**Professors:** Mario Giraldo & Manuel Ortiz
**Universidad EIA**

> Status: under active development. The robot homes, follows point-to-point and trajectory commands, runs pick & place cycles with a vacuum nozzle, and can be driven with a Wiimote.

---

## System overview

```
 iPad / PC browser ──HTTP──▶ scara_web (NiceGUI + ROS 2 Jazzy) ──ROS 2──▶ micro-ROS agent ──UART 115200──▶ ESP32
                                                                                                 │
                                                       3 DC motors (TB6612FNG) + encoders + end stops + servo + vacuum pump
```

- **ESP32 (ESP-IDF + micro-ROS):** homing, inverse kinematics, joint control, trajectory profiles, end effector and state observers. The control loop runs on core 1 every 8 ms and keeps running even if the USB link drops.
- **PC (ROS 2 Jazzy):** the web interface (`scara_web`), the micro-ROS agent and, optionally, the Wiimote node.

## Repository layout

| Path | Contents |
|---|---|
| `firmware/` | ESP32 firmware (ESP-IDF project) |
| `firmware/main/main.c` | Application logic and the micro-ROS interface (topics below) |
| `firmware/components/motors` | PWM, direction and end stops. Pin map in `include/pins.h` |
| `firmware/components/encoders` | 3 quadrature encoders read with the PCNT peripheral (x4) |
| `firmware/components/homing_pro` | Homing (double touch + calibration). This is the one the GUI uses |
| `firmware/components/kinematics` | Direct and inverse kinematics |
| `firmware/components/control` | Joint control: PD (`control.c`) or cascaded RST (`controlRST.c`), plus trajectory profiles (`profile.c`) |
| `firmware/components/effector` | Vacuum pump and nozzle rotation servo |
| `firmware/components/observers` | Luenberger observer + Kalman filter for each motor (current and speed) |
| `firmware/components/motor_test` | Open-loop motor test (circuit test tab) |
| `ros2_ws/src/scara_app` | ROS 2 package: web GUI, desktop GUI (PyQt5), trajectory planner, Wiimote control |
| `matlab/` | Identification, controller design (`NPI_Proyecto.mlx`), kinematics (`DKeIKmlx.mlx`) and observers (`LabIV_Sate_Observers.mlx`) |
| `tools/` | One-command launcher (`scara.launch.py` + `scara_prep.sh`) and patch scripts |

## Hardware

ESP32 DevKit V1 (30 pins), 2× TB6612FNG with STBY tied high, Pololu 37D 50:1 motors with 64 CPR (x4) encoders, 12 V supply. **`firmware/components/motors/include/pins.h` is the single source of truth for pins.** The PCB must match it.

| Function | GPIO |
|---|---|
| M1 IN1 / IN2 / PWM | 25 / 26 / 27 |
| M2 IN1 / IN2 / PWM | 21 / 19 / 18 |
| M3 (Z screw) IN1 / IN2 / PWM | 23 / 22 / 13 |
| Encoder 1 A / B | 36 / 39 |
| Encoder 2 A / B | 34 / 35 |
| Encoder 3 A / B | 32 / 33 |
| End stops M1 L / R | 5 / 17 |
| End stop M2 L (only one) | 4 |
| End stops M3 UP / DOWN | 14 / 15 |
| Nozzle servo (SG90, 50 Hz) | 2 |
| Vacuum pump (via MOSFET) | 16 |
| micro-ROS UART (do not use) | 1 / 3 |

Notes:
- GPIO 34–39 are input only and have no internal pull-ups. Encoder signals must be 3.3 V.
- Power the servo from a separate 5 V supply, with a common GND.
- Put a flyback diode across the pump.

## Quick start (one command)

Requirements:
- ROS 2 Jazzy.
- ESP-IDF, by default in `~/esp/esp-idf`.
- The micro-ROS agent built in `~/scara_ws/microros_ws`.
- NiceGUI: `pip install nicegui --break-system-packages`.

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ~/scara_ws/tools/scara.launch.py
```

This builds and flashes the ESP32, builds `scara_app`, then opens the micro-ROS agent and the web interface. If any step fails, it stops there. **Ctrl+C** closes everything.

| Argument | Default | Use |
|---|---|---|
| `zip:=<path>` | none | Unzip a set of changes into `~/scara_ws` first. Only the files it brings are touched, and `idf.py reconfigure` runs automatically if it adds a new component |
| `flash:=false` | `true` | Skip the firmware build and flash (fast start) |
| `web:=false` | `true` | Skip `colcon build` of `scara_app` |
| `port:=` | `/dev/ttyUSB0` | ESP32 serial port, used for flashing and for the agent |
| `baud:=` | `115200` | Agent baud rate |
| `idf:=<export.sh>` | `$IDF_PATH` or `~/esp/esp-idf` | ESP-IDF environment |

Examples:
```bash
ros2 launch ~/scara_ws/tools/scara.launch.py zip:=~/Downloads/Observadores_v1.zip
ros2 launch ~/scara_ws/tools/scara.launch.py flash:=false
```

ESP-IDF runs in a clean environment without ROS sourced, because the micro-ROS ESP-IDF component fails if ROS 2 is loaded in the same shell. The full build log goes to `firmware/build_last.log`.

Then open `http://<PC-IP>:8080` on the iPad or PC. The IP comes from `hostname -I`, and the port can be changed with the `SCARA_WEB_PORT` environment variable.

## Manual start (three terminals)

The agent must be **closed** while flashing, because both use the same port.

```bash
# 1. Firmware (with ESP-IDF loaded, WITHOUT ROS sourced)
cd ~/scara_ws/firmware
idf.py build
idf.py -p /dev/ttyUSB0 flash

# 2. micro-ROS agent
source /opt/ros/jazzy/setup.bash
source ~/scara_ws/microros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

# 3. Web interface
source /opt/ros/jazzy/setup.bash && cd ~/scara_ws/ros2_ws
colcon build --packages-select scara_app && source install/setup.bash
ros2 run scara_app scara_web          # or: ros2 run scara_app scara_gui (desktop, PyQt5)
```

For the Wiimote, also run `ros2 run scara_app wiimote_node`. See `ros2_ws/src/scara_app/WIIMOTE.md`.

## Web interface tabs

The header always shows the ESP32 connection, the current state and a **STOP** button (Esc on a keyboard).

| Tab | What it does |
|---|---|
| **Homing** | Runs homingPRO (motor 1 → 2 → 3), which zeroes the encoders and calibrates the ranges |
| **Punto a punto** | Sends a Cartesian target (x, y, z); the ESP32 solves the IK |
| **DK** | Moves by joint angles (th1, th2, z) |
| **Trayectoria** | Block editor (GO TO POSE, PICK, PLACE, ROTATE…) that validates and runs a pick & place cycle with quintic profiles generated on the ESP32. See `TRAYECTORIA_LEEME.md` |
| **Prueba de circuito** | End stops, encoders, open-loop motor runs, servo and pump |
| **Wiimote** | Drive the robot with a Wiimote. Starts in simulation mode; B is the dead-man |
| **Observadores** | Live plots per motor: estimated current (Kalman vs Luenberger) and speed (measured vs Luenberger vs Kalman). See `OBSERVADORES_LEEME.md` |

**Dead-man switch:** the motors only move while the operator holds the on-screen button or the space bar. The browser sends a heartbeat every 100 ms, and the firmware stops after 300 ms without one.

Leaving **Trayectoria** cancels the cycle, and leaving **Wiimote** disarms it. **Observadores** is the exception: you can switch to it without stopping anything, and it shows the same dead-man as the tab you came from.

## ROS 2 interface (ESP32 ↔ PC)

| Topic | Type | Direction | Content |
|---|---|---|---|
| `/scara/cmd` | String | PC → ESP32 | `start`, `resume`, `stop`, `release`, `dm1`/`dm0` (dead-man), `vac1`/`vac0`, `rot ±N`, `servo N`, `ff <gain>`, `obs1` (observer heartbeat) |
| `/scara/target` | Float32MultiArray | PC → ESP32 | `[x, y, z, d1..d6, av, codo]`, a jump to a target. With `[..., T, id]` the ESP32 interpolates to the goal in T seconds |
| `/scara/test_cmd` | String | PC → ESP32 | `run <motor> <L\|R> <%> <s>`, `stop <motor>`, `stop all` |
| `/scara/status` | String | ESP32 → PC | `IDLE`, `HOMING M1 SEEK_L`, `HOMED`, `ABORTED …`, `ERROR …` |
| `/scara/ik` | String | ESP32 → PC | IK result (`OK …` / `ERR … reason=…`) |
| `/scara/limits` | String | ESP32 → PC | End stops, encoders, joint positions, calibration, arrival, pump, servo, per-motor error and voltage |
| `/scara/obs` | Float32MultiArray | ESP32 → PC | 25 Hz, best effort: `[n, M1: w_meas w_luen w_kal i_luen i_kal, M2…, M3…]`. Only published while someone is viewing the Observadores tab |

## Configuration knobs

| What | Where |
|---|---|
| Controller: PD or cascaded RST | `CONTROL_USE_RST` in `firmware/components/control/include/control.h`. The build prints which one is compiled |
| Controller coefficients | `control.c` / `controlRST.c` (from MATLAB `tfdata`) |
| Trajectory speed and acceleration limits | `V_MAX_*`, `A_MAX_*` in `ros2_ws/src/scara_app/scara_app/trajectory.py` |
| Servo pulse range, pump active level | `firmware/components/effector/effector.c` |
| Observer model and tuning per motor (A, B, fac, Q, R) | `OBS_PARAMS[]` in `firmware/components/observers/observers.c`. L and K are recomputed at boot |
| Robot geometry (d1…d6, av) | `ROBOT_DEFAULTS`, sent with every target from the GUI |

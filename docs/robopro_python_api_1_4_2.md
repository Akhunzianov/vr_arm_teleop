# RoboPro RC Python API 1.4.2 Compact Reference

Source material: `Руководство_по_программированию_API_Python_1_4_2.pdf` and `python_api_1.4.2.zip`.
This is an agent-oriented quick reference, not a full translation of the manual.

## Scope And Safety

Python API controls RoboPro RC-series collaborative robots. Use it only from code that has an explicit robot IP, conservative speeds, clear connection ownership, and external safety supervision.

Qualified operators must handle real hardware. The API can move the robot, change controller state, set I/O, and configure wrist peripherals.

## Install

Python: `3.10` through `3.14`.

Package install:

```bash
pip install /path/to/python_api_1.4.2.zip
```

Main non-stdlib dependencies: `numpy`, `scipy`, `pymodbus`.

Imports:

```python
from API import RobotApi
from API.coords import CoordinateSystem, calculate_plane_from_points, convert_position_orientation
from API import types
from API.io import WristRS485, WristModbusRS485Client
```

## Connection Model

The controller exposes two sockets:

- RTD socket: real-time state stream, up to five simultaneous clients.
- command socket: commands/configuration, only one controlling client at a time.

Connection modes:

- `read_only=True`: RTD only. Use for monitoring while Pulse or another client owns control.
- `read_only=False`: full connection. Required for movement, state changes, settings, output writes, and most configuration.

If a method below is marked `[RO]`, it is available in read-only mode. `[FULL]` needs full command access. Unmarked coordinate helpers are local math and do not require robot connection.

## Minimal Patterns

Read-only monitoring:

```python
with RobotApi("192.168.0.12", read_only=True) as robot:
    joints = robot.motion.get_actual_position(orientation_units="deg", position_format="joints")
    tcp = robot.motion.get_actual_position(orientation_units="deg", position_format="tcp")
    safety = robot.safety_status.get()
```

Full-control motion:

```python
with RobotApi("192.168.0.12") as robot:
    robot.controller_state.set("off")
    robot.payload.set(mass=0.0, tcp_mass_center=(0, 0, 0))
    robot.tool.set((0, 0, 0.14329, 0, 0, 0), units="deg")
    robot.motion.scale_setup.set(velocity=0.3, acceleration=0.3)
    robot.controller_state.set("run", await_sec=120)

    robot.motion.joint.add_new_waypoint(
        angle_pose=(0, -115, 120, -100, -90, 0),
        speed=30,
        accel=60,
        blend=0,
        units="deg",
    )
    robot.motion.mode.set("move")
    robot.motion.wait_waypoint_completion(0)
```

Temporarily escalate from read-only:

```python
robot = RobotApi("192.168.0.12", autoconnect=False)
robot.connect(read_only=True)
with robot.connected(read_only=False):
    robot.controller_state.set("run")
robot.disconnect()
```

## Object Map

`RobotApi` is the entry point:

- `robot.safety_status`: safety status.
- `robot.controller_state`: controller state.
- `robot.controller_gravity`: gravity/orientation vector.
- `robot.motion`: movement and kinematics.
- `robot.io`: controller I/O.
- `robot.wrist`: wrist board I/O and RS-485.
- `robot.tool`: TCP/tool center point.
- `robot.payload`: payload mass and center.
- `robot.diagnostics`: robot diagnostics.

`robot.motion` contains:

- `motion.mode`: motion mode.
- `motion.scale_setup`: global velocity/acceleration scaling.
- `motion.joint`: joint-space movement.
- `motion.linear`: Cartesian linear movement.
- `motion.advanced`: MoveL/MoveP/MoveC/MoveJ advanced movement.
- `motion.kinematics`: forward/inverse kinematics.

`robot.io` contains `digital` and `analog`.

`robot.wrist` exposes properties `digital`, `analog`, and `rs_485`. `analog` requires wrist mode `"analog_in"`; `rs_485` requires wrist mode `"rs485"`.

## Common Types

Positions:

- TCP pose: `(X, Y, Z, Rx, Ry, Rz)`, meters plus orientation in `"deg"` or `"rad"`.
- Joint pose: six joint angles in `"deg"` or `"rad"`.
- `PositionOrientation`: list/tuple of floats, usually length 6.

Indexes and literals:

```python
AngleUnits = "deg" | "rad"
PositionFormat = "joints" | "tcp"
ReferenceFrame = "base" | "tcp"
JogAxis = "X" | "Y" | "Z" | "Rx" | "Ry" | "Rz"
JogDirection = "+" | "-"
JointIndex = 0..5
DigitalIndex = 0..23
DigitalSafetyIndex = 0..7
AnalogIndex = 0..3
DigitalWristIndex = 0..1
AnalogWristIndex = 0..3
PowerUnits = "mA" | "V"
CompareSigns = ">" | "<"
ControllerStateName = "on" | "off" | "run"
MotionModeName = "move" | "move_adv" | "pause" | "hold"
SafetyStatusName = "recovery" | "normal" | "reduced" | "safeguard_stop"
WristModeName = "off" | "rs485" | "analog_in" | "nc" | "gnd"
WristInputActivationType = "hold" | "trigger"
```

Digital input functions:

```python
"no_func" | "move" | "move_advhold" | "pause" | "zero_gravity" | "run" | "move_to_home"
```

Digital output functions:

```python
"no_func" | "no_move_signal_false" | "no_move_signal_true"
"move_status_signal_true_false" | "run_signal_true" | "warning_signal_true" | "error_signal_true"
```

Important data objects:

- `types.RobotInfo`: `robot_model`, `client_version`, `dh_model`.
- `types.DhModelParams`: `alpha`, `a`, `d`, `theta`, `offset`.
- `types.JointAngleDiscrepancy`: `joint_number`, `allowed_discrepancy`, `actual_position`, `saved_position`.
- `types.Response`: RS-485 result with `return_code`, `raw_data`, `is_ok`, `has_error`.
- `types.Rs485ReturnCodes`: `ok`, `timeout`, `tx_failed`, `rx_no_response`, `no_wrist_board`, `wrong_wrist_mode`, `op_in_progress`, `failure`, `overflow`, `not_init`.

## RobotApi

Constructor:

```python
RobotApi(
    ip="127.0.0.1",
    ignore_controller_exceptions=False,
    read_only=False,
    autoconnect=True,
    timeout=5,
    enable_logger=False,
    enable_logfile=False,
    logger=None,
    logfile_path=None,
    logfile_name=None,
    logfile_level=None,
    log_std_level=None,
    show_std_traceback=False,
)
```

Methods:

| Method | Mode | Purpose |
| --- | --- | --- |
| `connect(read_only=False) -> bool` | disconnected ok | Connect or switch mode. |
| `connected(read_only=False)` | disconnected ok | Context manager that restores previous connection state on exit. |
| `is_connected() -> bool` | disconnected ok | Current connection state. |
| `disconnect() -> bool` | disconnected ok | Close connection safely. |
| `get_robot_info() -> RobotInfo` | `[FULL]` | Robot model, client/core/protocol version, DH model. |
| `save() -> bool` | `[FULL]` | Persist user settings after power cycle. |
| `set_disconnection_callback(callback)` | `[FULL]` | Handler for unexpected disconnect. |

Use `with RobotApi(ip) as robot:` when possible.

## State And Safety

| Object | Method | Mode | Purpose |
| --- | --- | --- | --- |
| `robot.safety_status` | `get() -> str` | `[RO]` | Current safety status. |
| `robot.safety_status` | `wait(status, await_sec=-1) -> bool` | `[RO]` | Wait for safety status. |
| `robot.controller_state` | `get() -> str` | `[RO]` | Current controller state. |
| `robot.controller_state` | `set(state, await_sec=...) -> bool` | `[FULL]` | Set `"on"`, `"off"`, or `"run"`. |
| `robot.controller_state` | `set_confirm_position_callback(callback=None) -> bool` | `[FULL]` | Confirm mismatch between saved and actual joint positions. |
| `robot.controller_gravity` | `get() -> tuple[float, float, float] \| None` | `[FULL]` | Active gravity vector. |
| `robot.controller_gravity` | `set((x, y, z)) -> bool` | `[FULL]` | Configure controller orientation/gravity vector. |

`await_sec=-1` usually means wait forever; `0` usually means single immediate check.

## Tool And Payload

| Object | Method | Mode | Purpose |
| --- | --- | --- | --- |
| `robot.tool` | `set(tool_end_point, units=None) -> bool` | `[FULL]` | Set TCP relative to flange. |
| `robot.tool` | `get(units=None) -> PositionOrientation` | `[FULL]` | Get TCP offset. |
| `robot.payload` | `set(mass, tcp_mass_center) -> bool` | `[FULL]` | Set payload mass and center of mass. |
| `robot.payload` | `get() -> tuple[float, tuple]` | `[FULL]` | Get payload parameters. |
| `robot.payload` | `using(mass=None, tcp_mass_center=None)` | `[FULL]` | Temporary payload context manager. |

## Motion

Global methods:

| Method | Mode | Purpose |
| --- | --- | --- |
| `motion.set_motion_config(units=None, joint_speed=None, joint_acceleration=None, linear_speed=None, linear_acceleration=None, blend=None)` | local config | Set defaults for later movement commands. |
| `motion.get_actual_position(orientation_units=None, position_format="tcp", coordinate_system=None)` | `[RO]` | Current TCP or joint pose. |
| `motion.get_last_saved_position(orientation_units=None, position_format="joints", coordinate_system=None)` | `[FULL]` | Last saved position. |
| `motion.check_waypoint_completion(waypoint_count=0) -> bool` | `[RO]` | Non-blocking check for buffered waypoints. |
| `motion.wait_waypoint_completion(waypoint_count=0, await_sec=-1) -> bool` | `[RO]` | Wait until buffer fill is `<= waypoint_count`. |
| `motion.get_home_pose(units=None) -> PositionOrientation` | `[FULL]` | Current home pose. |
| `motion.set_home_pose(angle_pose, units=None) -> bool` | `[FULL]` | Set home pose in joint space. |
| `motion.move_to_home_pose()` | `[FULL]` | Add/move to home pose. |
| `motion.free_drive(enable=True) -> bool` | `[FULL]` | Enable/disable manual Free Drive. |
| `motion.is_point_reachable(tcp_pose, angle_pose=None, orientation_units=None, coordinate_system=None) -> bool` | `[FULL]` | Reachability check. |
| `motion.simple_joystick(coordinate_system=None) -> bool` | `[FULL]` | Launch built-in GUI joystick. |

Motion mode:

| Method | Mode | Purpose |
| --- | --- | --- |
| `motion.mode.get() -> str` | `[RO]` | Current motion mode. |
| `motion.mode.set(mode, await_sec=...) -> bool` | `[FULL]` | Set `"move"`, `"move_adv"`, `"pause"`, `"hold"`. |
| `motion.mode.check_warning_status() -> str` | `[RO]` | Motion warnings. |

Scaling:

| Method | Mode | Purpose |
| --- | --- | --- |
| `motion.scale_setup.set(velocity=1, acceleration=1) -> bool` | `[FULL]` | Global velocity/acceleration multipliers. |
| `motion.scale_setup.get() -> tuple[float, float] \| None` | `[FULL]` | Current multipliers. |

Joint motion:

| Method | Mode | Purpose |
| --- | --- | --- |
| `motion.joint.add_new_waypoint(angle_pose=None, tcp_pose=None, speed=None, accel=None, blend=None, units=None, coordinate_system=None) -> bool` | `[FULL]` | Add joint-space waypoint. Use either `angle_pose` or `tcp_pose`. |
| `motion.joint.get_actual_position(units=None) -> PositionOrientation` | `[RO]` | Current joint angles. |
| `motion.joint.get_last_saved_position(units=None)` | `[FULL]` | Last saved joint pose. |
| `motion.joint.jog_once(joint_index, jog_direction) -> bool` | `[FULL]` | Step one joint. |

Linear motion:

| Method | Mode | Purpose |
| --- | --- | --- |
| `motion.linear.add_new_waypoint(tcp_pose, speed=None, accel=None, blend=None, orientation_units=None, coordinate_system=None) -> bool` | `[FULL]` | Add linear TCP waypoint. |
| `motion.linear.add_new_offset(waypoint, offset, coordinate_system=None, speed=None, accel=None, blend=None, orientation_units=None) -> bool` | `[FULL]` | Add waypoint computed from base pose plus offset. |
| `motion.linear.get_actual_position(orientation_units=None, coordinate_system=None)` | `[RO]` | Current TCP pose. |
| `motion.linear.jog_once(jog_axis, jog_direction) -> bool` | `[FULL]` | Cartesian jog step. |
| `motion.linear.set_jog_param_in_tcp(coordinate_system) -> bool` | `[FULL]` | Configure TCP jogging reference. |

Advanced motion:

| Method | Mode | Purpose |
| --- | --- | --- |
| `motion.advanced.add_movel_waypoint(tcp_pose, translation_speed=None, translation_accel=None, rotation_speed=None, rotation_accel=None, blend=None, orientation_units=None, coordinate_system=None) -> bool` | `[FULL]` | Add MoveL segment. |
| `motion.advanced.add_movep_waypoint(tcp_pose, translation_speed=None, rotation_speed=None, translation_accel=None, rotation_accel=None, blend=None, orientation_units=None, coordinate_system=None) -> bool` | `[FULL]` | Add process MoveP segment. |
| `motion.advanced.add_movec_waypoint(tcp_pose_1, tcp_pose_2, translation_speed=None, rotation_speed=None, translation_accel=None, rotation_accel=None, blend=None, orientation_units=None, coordinate_system=None) -> bool` | `[FULL]` | Add circular MoveC segment through two TCP poses. |
| `motion.advanced.add_movej_waypoint(joints_pose, joints_speed=None, joints_accel=None, blend=0.0, orientation_units=None) -> bool` | `[FULL]` | Add MoveJ by joints. |
| `motion.advanced.add_movej_tcp_waypoint(tcp_pose, init_joints_pose=None, joints_speed=None, joints_accel=None, ik_solution_id=-1, blend=0.0, orientation_units=None, coordinate_system=None) -> bool` | `[FULL]` | Add MoveJ target by TCP pose. |

Kinematics:

| Method | Mode | Purpose |
| --- | --- | --- |
| `motion.kinematics.get_forward(angle_pose, units=None, coordinate_system=None)` | `[FULL]` | Joint pose to TCP pose. |
| `motion.kinematics.get_inverse(tcp_pose, angle_pose=None, orientation_units=None, coordinate_system=None, get_all=False)` | `[FULL]` | TCP pose to joint solution(s). |

Typical movement sequence:

```python
robot.motion.joint.add_new_waypoint((0, -90, 90, -90, -90, 0), units="deg")
robot.motion.mode.set("move")
robot.motion.wait_waypoint_completion(0)
```

For advanced waypoints, start with `motion.mode.set("move_adv")`.

## Controller I/O

Digital I/O:

| Method | Mode | Purpose |
| --- | --- | --- |
| `io.digital.get_input(index) -> bool` | `[RO]` | Digital input state. |
| `io.digital.get_safety_input(index) -> bool` | `[RO]` | Safety input state. |
| `io.digital.get_output(index) -> bool` | `[RO]` | Digital output state. |
| `io.digital.set_output(index, value) -> bool` | `[FULL]` | Set output. |
| `io.digital.wait_input(index, value, await_sec=-1) -> bool` | `[RO]` | Wait for one input state. |
| `io.digital.wait_any_input(await_sec=-1) -> bool` | `[RO]` | Wait for any input change. |
| `io.digital.set_input_function(index, function) -> bool` | `[FULL]` | Assign automatic input function. |
| `io.digital.get_input_functions(index=None)` | `[FULL]` | Get input function(s). |
| `io.digital.set_output_function(index, function) -> bool` | `[FULL]` | Assign automatic output function. |
| `io.digital.get_output_functions(index=None)` | `[FULL]` | Get output function(s). |
| `io.digital.get_safety_input_functions()` | `[FULL]` | Get safety input functions. |

Analog I/O:

| Method | Mode | Purpose |
| --- | --- | --- |
| `io.analog.get_input(index) -> tuple[int, float]` | `[RO]` | Analog input value. |
| `io.analog.set_output(index, value, units) -> bool` | `[FULL]` | Set analog output in `"mA"` or `"V"`. |
| `io.analog.wait_input(index, threshold_value, greater_or_less, await_sec=-1) -> bool` | `[RO]` | Wait for threshold crossing. |

## Wrist Board

Mode control:

| Method | Mode | Purpose |
| --- | --- | --- |
| `wrist.get_mode() -> str` | `[RO]` | Current wrist mode. |
| `wrist.set_mode(mode, await_sec=...) -> bool` | `[FULL]` | Set `"off"`, `"rs485"`, `"analog_in"`, `"nc"`, or `"gnd"`. |

Digital wrist I/O:

| Method | Mode | Purpose |
| --- | --- | --- |
| `wrist.digital.get_input(index) -> bool` | `[RO]` | Wrist input state. |
| `wrist.digital.get_output(index) -> bool` | `[RO]` | Wrist output state. |
| `wrist.digital.set_output(index, value) -> bool` | `[FULL]` | Set wrist output. |
| `wrist.digital.wait_input(index, value, await_sec=-1) -> bool` | `[RO]` | Wait for one input. |
| `wrist.digital.wait_any_input(await_sec=-1) -> bool` | `[RO]` | Wait for any wrist input change. |
| `wrist.digital.set_input_function(index, function) -> bool` | `[FULL]` | Assign wrist input function. |
| `wrist.digital.get_input_functions(index=None)` | `[FULL]` | Get wrist input function(s). |
| `wrist.digital.set_output_function(index, function) -> bool` | `[FULL]` | Assign wrist output function. |
| `wrist.digital.get_output_functions(index=None)` | `[FULL]` | Get wrist output function(s). |
| `wrist.digital.set_active_output(wrist_index, output_index, activation_type) -> bool` | `[FULL]` | Let wrist input control controller output. |

Analog wrist I/O requires wrist mode `"analog_in"`:

| Method | Mode | Purpose |
| --- | --- | --- |
| `wrist.analog.configure_input(index, units) -> bool` | `[FULL]` | Configure analog input units. |
| `wrist.analog.get_input(index) -> tuple[float, PowerUnits]` | `[RO]` | Read value and current unit. |
| `wrist.analog.get_input_in_units(index, units) -> float \| None` | `[FULL]` | Read converted value. |
| `wrist.analog.wait_input(index, threshold_value, greater_or_less, await_sec=-1) -> bool` | `[RO]` | Wait for threshold crossing. |
| `wrist.analog.wait_input_in_units(index, threshold_value, units, greater_or_less, await_sec=-1) -> bool` | `[FULL]` | Wait for threshold in requested units. |

## RS-485 And Modbus

Use wrist mode `"rs485"` before accessing `robot.wrist.rs_485`.

Raw RS-485:

```python
with RobotApi(ip, read_only=True) as robot:
    if robot.wrist.get_mode() != "rs485":
        with robot.connected(read_only=False):
            robot.wrist.set_mode("rs485", await_sec=30)

rs485 = WristRS485(host=ip)
with rs485.connected():
    resp = rs485.query(b"ID?\r\n")
    if resp and resp.is_ok:
        print(resp.raw_data)
```

`WristRS485(host, timeout=1, use_buffer=True, logger=None)` methods:

| Method | Purpose |
| --- | --- |
| `connect() -> bool` | Connect RS-485 gateway. |
| `connected()` | Context manager. |
| `is_connected() -> bool` | Connection state. |
| `reset() -> Response \| None` | Reset RS-485 service state. |
| `read(size=None) -> Response \| None` | Read incoming bytes. |
| `write(payload: bytes) -> Response \| None` | Send raw bytes. |
| `query(payload: bytes) -> Response \| None` | Write, then read. |
| `get_status() -> Response \| None` | RS-485 service status. |
| `clear_buffer() -> None` | Clear buffered frames. |
| `disconnect()` | Close connection. |

Modbus RTU over wrist RS-485:

```python
from API.io import WristModbusRS485Client

with WristModbusRS485Client(host=ip) as client:
    result = client.read_holding_registers(address=0x202)
```

Create with exactly one of:

```python
WristModbusRS485Client(host=ip, timeout=1.0)
WristModbusRS485Client(wrist_rs_485=robot.wrist.rs_485)
```

The Modbus client subclasses `pymodbus` sync client, so use standard pymodbus calls such as `read_holding_registers` and `write_register`.

## Coordinates

`CoordinateSystem(position_orientation, orientation_units=None, normalize_angles=True)` is local math and does not connect to the robot.

Methods:

| Method | Purpose |
| --- | --- |
| `set(position_orientation, orientation_units=None, normalize_angles=True) -> None` | Update coordinate system. |
| `get("pose")` | Return `(X, Y, Z, Rx, Ry, Rz)` in base frame. |
| `get("units")` | Return `"deg"` or `"rad"`. |
| `copy() -> CoordinateSystem` | Clone. |
| `with_units(orientation_units) -> CoordinateSystem` | Return converted copy. |
| `offset(dx=0, dy=0, dz=0) -> CoordinateSystem` | Shift along local axes. |
| `rotate(drx=0, dry=0, drz=0, orientation_units=None) -> CoordinateSystem` | Rotate around local axes. |
| `is_close(other, atol=1e-6) -> bool` | Compare with tolerance. |
| `distance_to(other) -> float` | Distance between origins. |
| `align_with_vector(target, up_vector=(0, 0, 1)) -> CoordinateSystem` | Align local X toward target. |
| `in_frame()` | Context manager: use as implicit coordinate system for API calls that accept `coordinate_system`. |

Coordinate helpers:

```python
calculate_plane_from_points(pO, pX, pY, orientation_units=None) -> PositionOrientation
convert_position_orientation(coordinate_system, position_orientation, orientation_units=None, to_local=False)
```

Example:

```python
base = CoordinateSystem((0, 0, 0, 0, 0, 0), orientation_units="deg")
part = base.offset(dy=0.2).rotate(drz=90)

with part.in_frame():
    local_tcp = robot.motion.get_actual_position(orientation_units="deg", position_format="tcp")
    robot.motion.linear.add_new_waypoint((0.1, 0, 0.2, 0, 0, 0), orientation_units="deg")
```

## Diagnostics

All diagnostics below are `[RO]`:

```python
diagnostics.get_controller_temperature() -> float
diagnostics.get_robot_voltage() -> float
diagnostics.get_robot_current() -> float
diagnostics.get_io_current() -> float
diagnostics.get_tool_current() -> float
diagnostics.get_joints_motor_temperatures() -> tuple[float, ...]
diagnostics.get_joints_controller_temperatures() -> tuple[float, ...]
diagnostics.get_joints_currents() -> tuple[float, ...]
diagnostics.get_joints_voltages() -> tuple[float, ...]
diagnostics.get_joints_torques() -> tuple[float, ...]
```

## Tools And Impulse Compatibility

`API.tools.sleep(await_sec=-1, frequency=0.005)` yields remaining time and is used by wait loops.

Impulse/Pulse compatibility helpers:

```python
from API.tools import load_impulse_vars, save_impulse_vars, impulse_vars, send_error_to_impulse

load_impulse_vars(*names)
save_impulse_vars(*names)
send_error_to_impulse(message)

with impulse_vars("a", "b") as vars:
    if vars.has_var("a"):
        ...
```

`add_impulse_vars` exists but is documented as unavailable in this release.

## Practical Rules For Agents

Prefer read-only connections for telemetry, diagnostics, position reads, and input waits.

Before any real movement:

1. Confirm the correct robot IP.
2. Use `robot.controller_state.set("off")` before changing payload/tool/motion setup.
3. Set payload and TCP.
4. Use low `motion.scale_setup` values first.
5. Set controller to `"run"`.
6. Add waypoint(s).
7. Start motion with `motion.mode.set("move")` or `motion.mode.set("move_adv")`.
8. Call `motion.wait_waypoint_completion(0)`.

Use `coordinate_system=` explicitly when mixing global and local frames. Do not rely on an implicit `CoordinateSystem.in_frame()` context inside large, multi-agent code unless the scope is tiny and obvious.

For wrist analog or RS-485, check `robot.wrist.get_mode()` first and switch mode only with full connection.

Never assume units. Pass `units=` or `orientation_units=` in motion code.

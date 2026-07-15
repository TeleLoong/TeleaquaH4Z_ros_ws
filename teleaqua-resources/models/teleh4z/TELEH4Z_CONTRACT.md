# TeleH4Z Actuator and Mode Contract

Status: authoritative Task 3 contract for TeleH4Z runtime integration.
Scope: names, indices, mode semantics, and target PX4↔Gazebo command contract.
Out of scope: final SDF rewiring, control allocator implementation, ROS mode request tool, and arm-follow bridge.

## 1. Vehicle mode semantics

- Water mode:
  - active propulsion family: 4 water thrusters
  - inactive propulsion family: 4 air props must remain neutral
  - arm pose target: retracted
  - arm targets: `arm_left_joint = 0`, `arm_right_joint = 0`

- Air mode:
  - active propulsion family: 4 air props
  - inactive propulsion family: 4 water thrusters must remain neutral
  - arm pose target: deployed
  - arm targets: `arm_left_joint = +pi/2`, `arm_right_joint = -pi/2`

PX4 is the only authoritative owner of requested mode and actual mode.
Gazebo must not infer mode from arm pose or propulsion activity.

## 2. Arm joint contract

| Joint | Parent | Axis | Retracted | Deployed | Source |
|---|---|---|---|---|---|
| `arm_left_joint` | `base_link` → `arm_left_link` | `1 0 0` | `0` | `+pi/2` | TeleH4Z SDF |
| `arm_right_joint` | `base_link` → `arm_right_link` | `-1 0 0` | `0` | `-pi/2` | TeleH4Z SDF |

Current SDF local control topic: `arm_cmd`.
This topic is provisional; Task 9 will make PX4 actual mode the true source that drives arm motion.

## 3. Air propulsion family (final contract)

Air propulsion follows the teleaquah8 multicopter pattern:
- Gazebo plugin family: `gz::sim::systems::MulticopterMotorModel`
- command topic: `command/motor_speed`
- indexing field: `motorNumber`
- TeleH4Z freezes a 4-motor Quad-X contract

### 3.1 PX4-style geometry summary for air rotors

This section deliberately mirrors the writing style of `4024_gz_teleaquah8`.

- User-provided air-prop positions are already treated as final TeleH4Z body-frame coordinates in the deployed-arm configuration.
- For Task 3 contract purposes, these coordinates are frozen directly as future PX4 `CA_ROTOR*_{P,A}{X,Y,Z}` geometry inputs.
- Air thrust direction follows the standard PX4 multicopter convention used by teleaquah8: `AX=0`, `AY=0`, `AZ=-1`.

```text
# --- Air rotors 0-3 (Quad-X) ---
# Positions below are already frozen in TeleH4Z body frame for the deployed-arm air mode.

# Motor 1 (right-front, CCW)
CA_ROTOR0_PX 0.1000
CA_ROTOR0_PY -0.1283
CA_ROTOR0_PZ 0.0660
CA_ROTOR0_AX 0.0
CA_ROTOR0_AY 0.0
CA_ROTOR0_AZ -1.0

# Motor 2 (left-front, CW)
CA_ROTOR1_PX 0.1000
CA_ROTOR1_PY 0.1283
CA_ROTOR1_PZ 0.0660
CA_ROTOR1_AX 0.0
CA_ROTOR1_AY 0.0
CA_ROTOR1_AZ -1.0

# Motor 3 (left-back, CCW)
CA_ROTOR2_PX -0.0920
CA_ROTOR2_PY 0.1283
CA_ROTOR2_PZ 0.0660
CA_ROTOR2_AX 0.0
CA_ROTOR2_AY 0.0
CA_ROTOR2_AZ -1.0

# Motor 4 (right-back, CW)
CA_ROTOR3_PX -0.0920
CA_ROTOR3_PY -0.1283
CA_ROTOR3_PZ 0.0660
CA_ROTOR3_AX 0.0
CA_ROTOR3_AY 0.0
CA_ROTOR3_AZ -1.0
```

### 3.2 Canonical air motor ordering

| motorNumber | Physical device | Joint | PX | PY | PZ | AX | AY | AZ | Rotation |
|---:|---|---|---:|---:|---:|---:|---:|---:|---|
| 0 | right-front air prop | `airprop_rightfront_joint` | 0.1000 | -0.1283 | 0.0660 | 0.0 | 0.0 | -1.0 | CCW |
| 1 | left-front air prop | `airprop_leftfront_joint` | 0.1000 | 0.1283 | 0.0660 | 0.0 | 0.0 | -1.0 | CW |
| 2 | left-back air prop | `airprop_leftback_joint` | -0.0920 | 0.1283 | 0.0660 | 0.0 | 0.0 | -1.0 | CCW |
| 3 | right-back air prop | `airprop_rightback_joint` | -0.0920 | -0.1283 | 0.0660 | 0.0 | 0.0 | -1.0 | CW |

Notes:
- This contract freezes standard Quad-X ordering and spin directions for TeleH4Z.
- Current TeleH4Z SDF still uses provisional `/airprop_spin` + joint-controller wiring; Task 5 must replace that with `command/motor_speed` + `motorNumber 0..3`.

## 4. Water propulsion family (final contract)

Water propulsion follows the teleaquah8 bidirectional-thruster pattern:
- Gazebo plugin family: `gz::sim::v8::systems::BiDirectionalMotorModel`
- command topics: `servo_0`, `servo_1`, `servo_2`, `servo_3`
- These thrusters are outside the air motor `command/motor_speed` family.

### 4.1 PX4-style geometry summary for water thrusters

This section also mirrors the `4024_gz_teleaquah8` writing style.

- Water-thruster positions come from the current TeleH4Z SDF.
- Coordinate conversion follows the same note used in `4024_gz_teleaquah8`:
  - `PX4(X, Y, Z) = SDF(X, -Y, -Z)`
- For Task 3 contract purposes, thrust-axis directions are frozen to the user-approved TeleH4Z convention below: right/left positive command -> forward surge; front/back positive command -> downward thrust.

```text
# --- Water thrusters 4-7 ---
# Coordinate conversion: PX4(X, Y, Z) = SDF(X, -Y, -Z)

# Rotor 4 / servo_0 (Right): SDF[-0.155, -0.036, -0.007866] -> PX4[-0.155, 0.036, 0.007866]
CA_ROTOR4_PX -0.1550
CA_ROTOR4_PY 0.0360
CA_ROTOR4_PZ 0.007866
CA_ROTOR4_AX 1.0
CA_ROTOR4_AY 0.0
CA_ROTOR4_AZ 0.0

# Rotor 5 / servo_1 (Front): SDF[0.1139, 0, 0.04] -> PX4[0.1139, 0, -0.04]
CA_ROTOR5_PX 0.1139
CA_ROTOR5_PY 0.0
CA_ROTOR5_PZ -0.0400
CA_ROTOR5_AX 0.0
CA_ROTOR5_AY 0.0
CA_ROTOR5_AZ 1.0

# Rotor 6 / servo_2 (Left): SDF[-0.155, 0.036, -0.007866] -> PX4[-0.155, -0.036, 0.007866]
CA_ROTOR6_PX -0.1550
CA_ROTOR6_PY -0.0360
CA_ROTOR6_PZ 0.007866
CA_ROTOR6_AX 1.0
CA_ROTOR6_AY 0.0
CA_ROTOR6_AZ 0.0

# Rotor 7 / servo_3 (Back): SDF[-0.10643, 0, 0.04] -> PX4[-0.10643, 0, -0.04]
CA_ROTOR7_PX -0.10643
CA_ROTOR7_PY 0.0
CA_ROTOR7_PZ -0.0400
CA_ROTOR7_AX 0.0
CA_ROTOR7_AY 0.0
CA_ROTOR7_AZ 1.0
```

### 4.2 Canonical water thruster ordering

| Rotor index | Command topic | Physical device | Joint | PX | PY | PZ | AX | AY | AZ |
|---:|---|---|---|---:|---:|---:|---:|---:|---:|
| 4 | `servo_0` | right water thruster | `waterprop_right_joint` | -0.1550 | 0.0360 | 0.007866 | 1.0 | 0.0 | 0.0 |
| 5 | `servo_1` | front water thruster | `waterprop_front_joint` | 0.1139 | 0.0 | -0.0400 | 0.0 | 0.0 | 1.0 |
| 6 | `servo_2` | left water thruster | `waterprop_left_joint` | -0.1550 | -0.0360 | 0.007866 | 1.0 | 0.0 | 0.0 |
| 7 | `servo_3` | back water thruster | `waterprop_back_joint` | -0.10643 | 0.0 | -0.0400 | 0.0 | 0.0 | 1.0 |

Notes:
- Ordering still mirrors the teleaquah8 channel order: right, front, left, back.
- Unlike teleaquah8, TeleH4Z freezes both X-axis thrusters with the same positive X thrust direction because positive command on both side thrusters must produce forward surge.
- Current TeleH4Z SDF does not yet implement the final bidirectional-thruster plugin/topic contract; Task 5 must add it.

## 5. Authoritative actuator family split

| Family | Members | Active in water | Active in air |
|---|---|---|---|
| Air propulsion | `airprop_rightfront_joint`, `airprop_leftfront_joint`, `airprop_leftback_joint`, `airprop_rightback_joint` | No | Yes |
| Water propulsion | `waterprop_right_joint`, `waterprop_front_joint`, `waterprop_left_joint`, `waterprop_back_joint` | Yes | No |
| Arm morphology | `arm_left_joint`, `arm_right_joint` | Retracted | Deployed |

## 6. Current implementation deviations to be fixed later

The current runtime TeleH4Z SDF (`Tools/simulation/gz/models/teleh4z/model.sdf`) is not yet compliant with this final contract:
- Air props are still driven by provisional `/airprop_spin` triggered publishers.
- Water thrusters do not yet expose the final `servo_0..servo_3` bidirectional-thruster contract.
- Arm joints use local `arm_cmd` topic, now driven by teleh4z_mode_manager via ros_gz_bridge (Task 9 done).

Those mismatches are expected at Task 3 and are the exact follow-up work for Task 5, Task 6, Task 8, and Task 9.

## 7. Source references

- Runtime model: `/home/user/PX4-Autopilot/Tools/simulation/gz/models/teleh4z/model.sdf`
- Design model: `/home/user/TeleH4Zmodel/model.sdf`
- TeleH4Z airframe baseline: `/home/user/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4026_gz_teleh4z`
- Reference model: `/home/user/PX4-Autopilot/Tools/simulation/gz/models/teleaquah8/model.sdf`

## 8. Mode API contract (Task 4)

### 8.1 Message types

Mode switching reuses the existing teleaquah8 hybrid message pair:

| Direction | ROS2 topic | uORB topic | Message type |
|---|---|---|---|
| Request (ROS→PX4) |  |  |  |
| Status (PX4→ROS) |  |  |  |

Message fields (identical for both Setpoint and Status):


### 8.2 Mode authority

- PX4  module is the sole authority
- Input priority: DDS > I2C > Default
- Default at boot:  (water) for TeleH4Z (set by 4026 airframe)
- ROS/script sends requests only; PX4 decides and publishes actual status

### 8.3 Arm control topic

| Direction | Topic | Message type | Bridge |
|---|---|---|---|
| ROS→Gazebo |  |  →  |  |

Both  and  share the same  topic.
Sending a single Float64 value drives both joints symmetrically:
- Axis definitions:  axis=(1,0,0),  axis=(-1,0,0)
-  → both arms deploy outward symmetrically
-  → both arms retract to body

### 8.4 Mode switch safety sequencing

**Water → Air** (deploy arms FIRST):
1. Publish  (deploy arms)
2. Wait for arms to reach position (3 sec default)
3. Publish setpoint  (switch PX4 to air)
4. Wait for PX4 status confirmation

**Air → Water** (switch PX4 FIRST):
1. Publish setpoint  (stop air props)
2. Wait for PX4 status confirmation
3. Publish  (retract arms)
4. Wait for arms to reach position

Rationale: always stop propellers before moving arms into their rotation plane.

### 8.5 Operator interface



### 8.6 Mode manager launch



This starts both  (arm topic) and  (orchestrator).

### 8.7 Source references

- Mode manager node: 
- Launch file: 
- PX4 mode module:  (compiled, source stripped)
- DDS topic config: 


## 8. Mode API contract (Task 4)

### 8.1 Message types

Mode switching reuses the existing teleaquah8 hybrid message pair:

| Direction | ROS2 topic | uORB topic | Message type |
|---|---|---|---|
| Request (ROS to PX4) | /fmu/in/vehicle_air_water_status_setpoint | vehicle_air_water_status_setpoint | VehicleAirWaterStatusSetpoint |
| Status (PX4 to ROS) | /fmu/out/vehicle_air_water_status | vehicle_air_water_status | VehicleAirWaterStatus |

Message fields (identical for both Setpoint and Status):

    uint64 timestamp
    uint8 uw_transit_mode    # 0=air, 1=water
    uint8 transform_status   # 0=complete, 1=transforming (reserved, not enforced in V1)

### 8.2 Mode authority

- PX4 air_water_status_publisher module is the sole authority
- Input priority: DDS > I2C > Default
- Default at boot: uw_transit_mode=1 (water) for TeleH4Z (set by 4026 airframe)
- ROS/script sends requests only; PX4 decides and publishes actual status

### 8.3 Arm control topic

| Direction | Topic | Message type | Bridge |
|---|---|---|---|
| ROS to Gazebo | /arm_cmd | std_msgs/msg/Float64 to gz.msgs.Double | ros_gz_bridge |

Both arm_left_joint and arm_right_joint share the same arm_cmd topic.
Sending a single Float64 value drives both joints symmetrically:
- Axis definitions: arm_left_joint axis=(1,0,0), arm_right_joint axis=(-1,0,0)
- +pi/2 -> both arms deploy outward symmetrically
- 0 -> both arms retract to body

### 8.4 Mode switch safety sequencing

Water to Air (deploy arms FIRST):
1. Publish arm_cmd = +pi/2 (deploy arms)
2. Wait for arms to reach position (3 sec default)
3. Publish setpoint {uw_transit_mode: 0} (switch PX4 to air)
4. Wait for PX4 status confirmation

Air to Water (switch PX4 FIRST):
1. Publish setpoint {uw_transit_mode: 1} (stop air props)
2. Wait for PX4 status confirmation
3. Publish arm_cmd = 0 (retract arms)
4. Wait for arms to reach position

Rationale: always stop propellers before moving arms into their rotation plane.

### 8.5 Operator interface

    # Request air mode
    ros2 topic pub --once /teleh4z/mode_request std_msgs/msg/String "{data: 'air'}"

    # Request water mode
    ros2 topic pub --once /teleh4z/mode_request std_msgs/msg/String "{data: 'water'}"

### 8.6 Mode manager launch

    ros2 launch teleh4z_manager mode_manager.launch.py

This starts both ros_gz_bridge (arm topic) and teleh4z_mode_manager (orchestrator).

### 8.7 Source references

- Mode manager node: /home/user/ros2_ws/src/teleh4z_manager/teleh4z_manager/mode_manager.py
- Launch file: /home/user/ros2_ws/src/teleh4z_manager/launch/mode_manager.launch.py
- PX4 mode module: air_water_status_publisher (compiled, source stripped)
- DDS topic config: /home/user/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml

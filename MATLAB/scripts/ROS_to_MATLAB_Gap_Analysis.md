# ROS (ME_557_Robot) → MATLAB (557-Robot-Arm-Matlab) Gap Analysis

## 1. Home / Zero Position — All Joints

**Both repos agree: the home (zero) position is the arm standing vertically straight up.**

| Convention | Home Value | Meaning |
|---|---|---|
| MoveIt (ROS URDF) | **0.0 rad** for all 5 joints | Zero-centered; the SRDF `<group_state name="home">` sets every joint to 0.0 |
| Arduino (physical servos) | **180° logical** for all 6 servos | Servos are homed to 180° on startup (`homeAllMotors()`) |
| MATLAB `robot_model_from_urdf` | `robot.home = zeros(5,1)` | Matches MoveIt convention |

The conversion between the two is:
```
logicalDeg = 180 + DIR_SIGN[i] * (moveit_rad - HOME_RAD[i]) * (180/π)
```
Since `HOME_RAD = [0, 0, 0, 0, 0]`, at MoveIt 0 rad → 180° logical → arm straight up.

---

## 2. Joint Limits — THE BIGGEST GAP

Your MATLAB URDF (`simple_robot_v2.urdf`) has **SolidWorks-exported offset limits** that are NOT zero-centered. The ROS URDF (`me557_pen.urdf`) has the **correct zero-centered operational limits** that MoveIt uses. **Your MATLAB code (`robot_model_from_urdf.m`) does NOT parse or use joint limits at all** — there is no limit checking in your IK solver.

### ROS URDF Joint Limits (zero-centered, radians) — USE THESE

| MoveIt Joint | Axis | Lower (rad) | Upper (rad) | Lower (deg) | Upper (deg) |
|---|---|---|---|---|---|
| Motor1_joint (Base) | [0 0 −1] | **−0.87265** | **+0.87265** | −50.0° | +50.0° |
| Motor2_L (Shoulder) | [0 0 +1] | **−1.04720** | **+1.04720** | −60.0° | +60.0° |
| Motor4_elb (Elbow) | [0 0 +1] | **−1.87623** | **+1.87623** | −107.5° | +107.5° |
| Motor5_wr (Wrist) | [0 0 −1] | **−0.95995** | **+0.95995** | −55.0° | +55.0° |
| Joint_EE (Pen Rot.) | [0 0 +1] | **−1.17810** | **+1.17810** | −67.5° | +67.5° |

### Your MATLAB URDF Limits (SolidWorks offset, radians) — FOR REFERENCE ONLY

| MATLAB Joint | Axis | Lower (rad) | Upper (rad) |
|---|---|---|---|
| RotatingBaseJoint | [0 0 −1] | 2.0944 | 3.8397 |
| ShoulderJoint | [0 0 +1] | 2.0944 | 4.18879 |
| ElbowJoint | [0 0 +1] | 1.3090 | 5.06145 |
| WristJoint | [0 0 −1] | 2.6180 | 4.5379 |
| PenJoint | [0 0 +1] | 1.4835 | 3.8397 |

The SolidWorks limits are offset by π (≈3.14159) from zero because the exporter defines "zero" differently. For your MATLAB IK, **use the ROS zero-centered limits**.

---

## 3. Physical Servo Calibration Data (Arduino → Missing from MATLAB)

This is the hardware-level mapping that your MATLAB code will eventually need for full 5-joint control. It lives in the Arduino `.ino` files but has no MATLAB equivalent.

### Per-Motor Table

| Servo ID | MoveIt Joint | Motor Type | Physical Offset (°) | Logical Lower (°) | Logical Upper (°) | Range (°) |
|---|---|---|---|---|---|---|
| 1 | Motor1_joint | MX-64 / XM | 117.0 | 120 | 220 | 100° |
| 2 | Motor2_L | MX-64 / XM | 187.0 | 120 | 240 | 120° |
| 3 | Motor3_R (mirror of 2) | MX-64 / XM | 189.0 | 120 | 240 | 120° |
| 4 | Motor4_elb | AX-12 | 150.0 | 40 | 240 | 200° |
| 5 | Motor5_wr | AX-12 | 150.0 | 100 | 260 | 160° |
| 6 | Joint_EE | AX-12 | 150.0 | 140 | 300 | 160° |

### Servo Hardware Details
- **AX-12** servos (model number 12): 10-bit resolution (0–1023), 300° physical range
- **MX/XM/XH** servos: 12-bit resolution (0–4095), 360° physical range
- Baud rate: **1,000,000** (1 Mbps)
- Protocol version: **1.0** (all sketches use `setPortProtocolVersion(1.0)`)

### Physical ↔ Logical Conversion
```
physicalDeg = logicalAngle + (physicalOffset - 180.0)
logicalAngle = physicalDeg - (physicalOffset - 180.0)
```

### MoveIt ↔ Logical Conversion
```
Direction signs:  MOVEIT_DIR_SIGN = [-1, +1, +1, -1, +1]
Home offsets:     MOVEIT_HOME_RAD = [ 0,  0,  0,  0,  0]

logicalDeg = 180.0 + DIR_SIGN[i] * (rad - HOME_RAD[i]) * (180/π)
moveit_rad = HOME_RAD[i] + ((logicalDeg - 180.0) * (π/180)) / DIR_SIGN[i]
```

---

## 4. Motor 2/3 Mirroring (Missing from MATLAB)

The shoulder uses a **parallel linkage**: Motor 2 (left) and Motor 3 (right) are mechanically coupled and must be driven as mirrors of each other. In the ROS URDF this is declared as:

```xml
<joint name="Motor3_R" type="revolute">
  <mimic joint="Motor2_L" multiplier="-1" offset="0" />
</joint>
```

In the Arduino code: `executeMove(partnerID, logicalToPhysical(partnerID, 360.0 - logicalAngle))`

**Your MATLAB repo treats this as a single ShoulderJoint with no mirroring logic.** This is fine for kinematics/IK but will matter when you send commands to real hardware — you must command both ID 2 AND ID 3 with mirrored values.

---

## 5. Motor 5 Safety Tuck (Missing from MATLAB)

The Arduino code has a critical safety behavior: **when Motor 4 (elbow) goes below 70° logical, Motor 5 (wrist) must be tucked to 100° to avoid physical collision.** This collision avoidance is completely absent from the MATLAB code.

```
if (motor4_logicalAngle < 70.0) → tuck motor5 to 100.0°
if (motor4_logicalAngle >= 70.0) → restore motor5 to previous angle
```

---

## 6. What You SHOULD Add to Your MATLAB Repository

### 6A. Joint Limit Checking (CRITICAL — add to `robot_model_from_urdf.m` or a new file)

Your `robot_model_from_urdf.m` parses the URDF but **never extracts `<limit>` tags**. Your IK solver can happily return joint angles outside the physical range. Create a MATLAB struct with the zero-centered limits:

```matlab
% robot_joint_limits.m — Add this file
function limits = robot_joint_limits()
% Zero-centered joint limits from the ROS/MoveIt URDF (me557_pen.urdf)
% These are the ACTUAL operational limits of the physical arm.
%
%  Joint order matches: Motor1_joint, Motor2_L, Motor4_elb, Motor5_wr, Joint_EE
%  (same order as jointNamesOrdered in main_writeboard_demo.m)

limits.lower = [ -0.87265;   % Joint 1 — Base rotation
                 -1.04720;   % Joint 2 — Shoulder
                 -1.87623;   % Joint 3 — Elbow
                 -0.95995;   % Joint 4 — Wrist
                 -1.17810 ]; % Joint 5 — End-effector rotation

limits.upper = [  0.87265;
                  1.04720;
                  1.87623;
                  0.95995;
                  1.17810 ];
end
```

Then add a clamp/check after every IK solve:

```matlab
function theta_clamped = clamp_joints(theta, limits)
    theta_clamped = max(limits.lower, min(limits.upper, theta));
    if any(theta_clamped ~= theta)
        warning('Joint limits violated — clamped.');
    end
end
```

### 6B. Servo Calibration Mapping (For hardware control)

```matlab
% servo_calibration.m — Add this file
function cal = servo_calibration()
% Hardware calibration from the Arduino firmware.
% Maps between MoveIt radians and physical servo commands.

cal.servoIDs     = [1,   2,   3,   4,   5,   6  ];
cal.physOffset   = [117, 187, 189, 150, 150, 150]; % degrees
cal.logicalLower = [120, 120, 120,  40, 100, 140]; % degrees
cal.logicalUpper = [220, 240, 240, 240, 260, 300]; % degrees

% MoveIt joint index → Servo ID mapping (5 MoveIt joints → 6 servos)
cal.moveitToServoID = [1, 2, 4, 5, 6];  % Motor3 (ID3) is mirrored from Motor2 (ID2)

% Direction signs: MoveIt positive → which logical direction
cal.dirSign = [-1, 1, 1, -1, 1];

% Servo models: 12 = AX-12 (300°/1023), otherwise MX/XM (360°/4095)
% Detected at runtime on real hardware, but for planning:
cal.isAX12 = [false, false, false, true, true, true]; % IDs 4,5,6 are AX-12
end
```

### 6C. Conversion Functions (For eventual hardware control)

```matlab
% moveit_rad_to_servo.m
function [goalPositions, servoIDs] = moveit_rad_to_servo(theta_rad, cal)
% Convert 5-element MoveIt joint radians to 6 servo goal positions.
% theta_rad: 5x1 vector [base; shoulder; elbow; wrist; ee]
% Returns goalPositions for IDs [1,2,3,4,5,6]

RAD2DEG = 180/pi;
logicalAngles = zeros(6,1);

for i = 1:5
    sid = cal.moveitToServoID(i);
    logDeg = 180.0 + cal.dirSign(i) * theta_rad(i) * RAD2DEG;
    idx = find(cal.servoIDs == sid);
    logDeg = max(cal.logicalLower(idx), min(cal.logicalUpper(idx), logDeg));
    logicalAngles(idx) = logDeg;
end

% Mirror: ID3 = 360 - ID2's logical angle
idx2 = find(cal.servoIDs == 2);
idx3 = find(cal.servoIDs == 3);
logicalAngles(idx3) = 360.0 - logicalAngles(idx2);

% Convert logical → physical → raw
goalPositions = zeros(6,1);
servoIDs = cal.servoIDs;
for i = 1:6
    physDeg = logicalAngles(i) + (cal.physOffset(i) - 180.0);
    if cal.isAX12(i)
        goalPositions(i) = round(constrain(physDeg, 0, 300) * (1023/300));
    else
        goalPositions(i) = round(constrain(physDeg, 0, 360) * (4095/360));
    end
end
end

function v = constrain(x, lo, hi)
    v = max(lo, min(hi, x));
end
```

### 6D. Collision / Self-Collision Pairs (from SRDF)

The MoveIt SRDF defines which link pairs to skip during self-collision checking (adjacent links):

```
base_link ↔ Link1       (Adjacent)
Link1     ↔ Link2_L     (Adjacent)
Link2_L   ↔ Link4       (Adjacent)
Link4     ↔ Link5       (Adjacent)
Link5     ↔ Link6       (Adjacent)
Link6     ↔ Pen_tip     (Adjacent)
base_link ↔ floor       (Adjacent)
```

If you ever add self-collision checking to your MATLAB planner, these are the pairs to SKIP.

---

## 7. Joint Name Mapping Between Repos

| # | MATLAB URDF Joint | ROS URDF Joint | Servo ID(s) |
|---|---|---|---|
| 1 | RotatingBaseJoint | Motor1_joint | 1 |
| 2 | ShoulderJoint | Motor2_L (+Motor3_R mimic) | 2 (+3 mirror) |
| 3 | ElbowJoint | Motor4_elb | 4 |
| 4 | WristJoint | Motor5_wr | 5 |
| 5 | PenJoint | Joint_EE | 6 |

Note: There is also a **PWM hobby servo on Arduino pin 3** (addressed as "ID 7" in the online Arduino sketch) that controls the pen up/down mechanism. It is NOT a Dynamixel and is controlled separately via `Servo.write(0–180)`, homed to 90°.

---

## 8. Additional ROS Features NOT in MATLAB (Info Only — Not MATLAB-Readable)

These are valuable features from the ROS repo that aren't directly portable to MATLAB as files, but whose logic you should be aware of:

- **OMPL Motion Planner config**: Uses RRTConnect and RRTstar with `longest_valid_segment_fraction = 0.005` — this means collision checking every 0.5% of each motion segment. Your MATLAB IK doesn't do any intermediate collision checking.

- **Planning Scene collision objects**: The ROS pipeline adds a board/table collision box before planning so the arm avoids hitting the writing surface during transit moves. Your MATLAB code has no equivalent.

- **Trajectory time parameterization**: MoveIt applies `AddTimeOptimalParameterization` to smooth trajectories with proper velocity/acceleration profiles. Your MATLAB trajectory is just waypoint-to-waypoint.

- **Velocity-aware playback (code 777)**: The online Arduino sketch uses per-waypoint timing (`dt_ms`) and per-joint velocities from MoveIt's time-optimal parameterization for smooth motion. The MATLAB pipeline doesn't generate timing information.

---

## 9. Summary: Priority Actions

1. **🔴 CRITICAL — Add joint limit checking** using the ROS zero-centered limits. Without this your IK can return physically impossible poses.

2. **🟠 HIGH — Add servo calibration data** (`physicalOffset`, `logicalLower/Upper`, `dirSign`, `moveit_to_servo_id` mapping) so your MATLAB pipeline can generate commands the Arduino actually understands.

3. **🟠 HIGH — Add Motor 2/3 mirroring logic** to your hardware interface for when you move beyond single-motor control.

4. **🟡 MEDIUM — Add Motor 5 safety tuck** logic to prevent wrist/elbow collision.

5. **🟡 MEDIUM — Add the ROS-style URDF** (`me557_pen.urdf`) alongside your existing URDF, or update `robot_model_from_urdf.m` to parse `<limit>` tags from whatever URDF it reads.

6. **🟢 LOW — Collision avoidance / planning scene** — would require a full collision checking library, probably overkill for MATLAB.

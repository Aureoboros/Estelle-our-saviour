# FTC Drive System - Complete Documentation

## Table of Contents

1. [Overview](#overview)
2. [Program Comparison](#program-comparison)
3. [ChassisTest - Diagnostic Mode](#chassistest---diagnostic-mode)
   - [Hardware Requirements](#hardware-requirements-chassistest)
   - [Core Features](#core-features-chassistest)
   - [Test Modes](#test-modes)
   - [Control Reference](#control-reference-chassistest)
   - [Odometry System](#odometry-system-chassistest)
   - [Use Cases](#use-cases-chassistest)
4. [JustMotion - Competition Mode](#justmotion---competition-mode)
   - [Hardware Requirements](#hardware-requirements-justmotion)
   - [Core Features](#core-features-justmotion)
   - [Speed Control System](#speed-control-system)
   - [Control Reference](#control-reference-justmotion)
   - [Odometry System](#odometry-system-justmotion)
   - [Use Cases](#use-cases-justmotion)
5. [Technical Specifications](#technical-specifications)
6. [Configuration Guide](#configuration-guide)
7. [Troubleshooting](#troubleshooting)
8. [Best Practices](#best-practices)

---

## Overview

This documentation covers two complementary FTC (FIRST Tech Challenge) robot control programs designed for mecanum drive robots:

- **ChassisTest**: Comprehensive diagnostic and testing tool
- **JustMotion**: Streamlined competition driving program

Both programs support dual-driver operation, odometry-based position tracking, and IMU integration for heading measurement.

---

## Program Comparison

| Feature | ChassisTest | JustMotion |
|---------|-------------|------------|
| **Primary Purpose** | Testing & Diagnostics | Competition Driving |
| **Test Modes** | 9 modes (individual motors, pairs, sides) | None - drive only |
| **Speed Options** | Slow mode toggle only | 3 presets + slow mode |
| **Field-Centric** | ❌ No | ✅ Yes (toggle) |
| **Odometry Type** | Dedicated wheels (2-wheel) | Drive motor encoders |
| **Driver 1 Power** | 50% max | 100% max |
| **Driver 2 Power** | 50% max | 50% max |
| **DPAD Function** | Mode selection | Precision movement |
| **Emergency Stop** | ✅ A button | ❌ No |
| **Heading Reset** | Full reset only | Full or heading-only |
| **Best For** | Hardware testing, calibration, diagnostics | Matches, practice driving |

---

## ChassisTest - Diagnostic Mode

### Hardware Requirements (ChassisTest)

#### Required Components
- **4 Drive Motors**: `frontLeftMotor`, `backLeftMotor`, `frontRightMotor`, `backRightMotor`
- **2 Odometry Encoders**: `xOdo` (strafe/sideways), `yOdo` (forward/back)
- **IMU Sensor**: REV Hub IMU configured as `imu`
- **Control Hub**: REV Robotics Control Hub or Expansion Hub

#### Hardware Configuration
```
Device Name         | Type           | Port
--------------------|----------------|------
frontLeftMotor      | DC Motor       | Motor Port 0
backLeftMotor       | DC Motor       | Motor Port 1
frontRightMotor     | DC Motor       | Motor Port 2
backRightMotor      | DC Motor       | Motor Port 3
xOdo                | DC Motor       | Motor Port (encoder only)
yOdo                | DC Motor       | Motor Port (encoder only)
imu                 | IMU            | I2C Port 0
```

### Core Features (ChassisTest)

#### Dual-Driver Mecanum Control
- Two gamepads can operate the robot simultaneously
- Automatic driver detection based on joystick activity
- Both drivers limited to 50% maximum power
- Independent control with smooth handoff between drivers

#### Position Tracking
- **Two-wheel odometry system** with dedicated encoder wheels
- Real-time position display (X, Y in inches)
- Heading display in degrees
- Accounts for odometry wheel offset from robot center
- Compensates for rotation effects during turns

#### Safety Features
- **Emergency stop** via A button (stops all motors immediately)
- **Brake mode** when motors idle
- **10% joystick deadzone** to prevent drift
- Visual indicators for active motors in test modes

### Test Modes

ChassisTest includes 9 distinct operating modes:

#### 1. Normal Drive Mode
Standard mecanum drive with all four wheels operational.
- **Controls**: Left stick Y (forward/back), Right stick X (strafe), Triggers (rotate)
- **Status**: Default mode on startup

#### 2-5. Individual Motor Tests
Test each motor independently using left stick Y axis.
- **TEST_FRONT_LEFT**: Front left motor only
- **TEST_BACK_LEFT**: Back left motor only  
- **TEST_FRONT_RIGHT**: Front right motor only
- **TEST_BACK_RIGHT**: Back right motor only
- **Indicator**: Active motor marked with `◄◄◄` in telemetry

#### 6-7. Motor Pair Tests
Test front or back motor pairs together.
- **TEST_FRONT_PAIR**: Both front motors
- **TEST_BACK_PAIR**: Both back motors

#### 8-9. Side Tests
Test left or right side motors together.
- **TEST_LEFT_SIDE**: Front left + back left
- **TEST_RIGHT_SIDE**: Front right + back right

### Control Reference (ChassisTest)

#### Gamepad 1 & 2 (Both Controllers)

| Control | Function | Mode |
|---------|----------|------|
| **Left Stick Y** | Forward/Backward | Normal + Test modes |
| **Right Stick X** | Strafe Left/Right | Normal mode only |
| **Left Trigger** | Rotate Left | Normal mode only |
| **Right Trigger** | Rotate Right | Normal mode only |
| **DPAD UP** | Switch to Normal Drive | All modes |
| **DPAD DOWN** | Cycle Individual Motor Tests | All modes |
| **DPAD LEFT** | Cycle Motor Pair Tests | All modes |
| **DPAD RIGHT** | Cycle Side Tests | All modes |
| **START** | Toggle Slow Mode (30%) | All modes |
| **BACK** | Reset IMU + Odometry | All modes |
| **A Button** | Emergency Stop All Motors | All modes |

### Odometry System (ChassisTest)

#### Configuration Details
```java
Wheel Diameter: 32mm (1.26 inches)
Encoder Resolution: 2000 counts per revolution
Counts per Inch: ~500 counts

Y Odometry Wheel Position:
  - 3.35 inches LEFT of center
  - 1 inch BACK from center
  
X Odometry Wheel Position:
  - 3.35 inches RIGHT of center
  - 1 inch BACK from center

Calculated Track Width: 6.7 inches
```

#### Odometry Algorithm
1. Read encoder deltas from both odometry wheels
2. Convert counts to inches of wheel movement
3. Account for wheel offset during robot rotation
4. Calculate robot-centric displacement
5. Convert to field-centric using IMU heading
6. Update global X, Y, heading values

### Use Cases (ChassisTest)

✅ **When to Use ChassisTest:**
- Initial robot setup and motor direction verification
- Diagnosing mechanical issues (wheels not moving, grinding)
- Testing individual motor functionality
- Calibrating odometry wheel positions
- Verifying encoder connections and readings
- Training new drivers on basic controls
- Pre-competition hardware checkout

❌ **Not Ideal For:**
- Actual competition matches
- High-speed driving practice
- Field-centric driving training

---

## JustMotion - Competition Mode

### Hardware Requirements (JustMotion)

#### Required Components
- **4 Drive Motors**: `frontLeftMotor`, `backLeftMotor`, `frontRightMotor`, `backRightMotor` (with encoders)
- **IMU Sensor**: REV Hub IMU configured as `imu`
- **Control Hub**: REV Robotics Control Hub or Expansion Hub

#### Hardware Configuration
```
Device Name         | Type           | Port
--------------------|----------------|------
frontLeftMotor      | DC Motor       | Motor Port 0
backLeftMotor       | DC Motor       | Motor Port 1
frontRightMotor     | DC Motor       | Motor Port 2
backRightMotor      | DC Motor       | Motor Port 3
imu                 | IMU            | I2C Port 0
```

**Note**: JustMotion uses drive motor encoders for odometry, eliminating the need for separate odometry wheels.

### Core Features (JustMotion)

#### Advanced Speed Control
- **Three speed presets**: Slow (30%), Medium (60%), Fast (100%)
- **Slow mode override**: START button forces 30% speed
- **Per-driver power limits**: Driver 1 at 100%, Driver 2 at 50%
- **Precision DPAD control**: 30% power for fine adjustments

#### Field-Centric Driving
- Toggle between robot-centric and field-centric modes
- In field-centric: forward always moves toward opponent's side
- Essential for consistent driving regardless of robot orientation
- Uses IMU heading for coordinate transformation

#### Flexible Resets
- **Full reset** (BACK): Position, heading, and encoders
- **Heading-only reset** (Left Bumper): Realign field-centric reference
- **Snap to 0°** (Right Bumper): Quick forward alignment (placeholder)

### Speed Control System

#### Speed Preset Matrix

| Mode | Button | Forward/Strafe | Rotation | Use Case |
|------|--------|----------------|----------|----------|
| **SLOW** | A | 30% | 30% | Precise scoring, alignment |
| **MEDIUM** | B | 60% | 60% | Controlled navigation |
| **FAST** | X | 100% | 100% | Transit, repositioning |
| **Slow Mode** | START | 30% | 30% | Emergency precision (overrides preset) |

#### DPAD Precision Control
- Fixed 30% power output regardless of preset
- Overrides joystick input when active
- Directions: UP (forward), DOWN (back), LEFT/RIGHT (strafe)

### Control Reference (JustMotion)

#### Gamepad 1 (Primary Driver)

| Control | Function | Notes |
|---------|----------|-------|
| **Left Stick Y** | Forward/Backward | Inverted from typical |
| **Right Stick X** | Strafe Left/Right | |
| **Left Trigger** | Rotate Left | Analog control |
| **Right Trigger** | Rotate Right | Analog control |
| **DPAD UP** | Precise Forward | 30% power |
| **DPAD DOWN** | Precise Backward | 30% power |
| **DPAD LEFT** | Precise Strafe Left | 30% power |
| **DPAD RIGHT** | Precise Strafe Right | 30% power |
| **A Button** | Speed: SLOW (30%) | Preset selection |
| **B Button** | Speed: MEDIUM (60%) | Preset selection |
| **X Button** | Speed: FAST (100%) | Preset selection |
| **Y Button** | Toggle Field-Centric | Changes control mode |
| **START** | Toggle Slow Mode | Overrides preset |
| **BACK** | Reset Position + Heading | Full reset |
| **Left Bumper** | Reset Heading Only | For field-centric |
| **Right Bumper** | Snap to 0° | (Future feature) |

#### Gamepad 2 (Secondary Driver)
- All controls identical to Gamepad 1
- Limited to 50% maximum power
- Useful for co-driver during endgame

### Odometry System (JustMotion)

#### Configuration Details
```java
Motor Type: REV HD Hex Motor
Encoder Resolution: 537.7 counts per revolution
Gear Reduction: 1.0 (direct drive)
Wheel Diameter: 4.0 inches
Counts per Inch: ~42.9 counts
```

#### Drive Encoder Odometry Algorithm
1. Read all four drive motor encoder deltas
2. Calculate average forward movement: `(FL + BL + FR + BR) / 4`
3. Calculate average strafe movement: `(-FL + BL + FR - BR) / 4`
4. Convert counts to inches
5. Use IMU heading for field-centric transformation
6. Update global X, Y position

**Advantages:**
- No additional hardware required
- Simple setup

**Disadvantages:**
- Less accurate during wheel slip
- Cannot distinguish between intended and unintended movement

### Use Cases (JustMotion)

✅ **When to Use JustMotion:**
- Competition matches (qualification, playoff, finals)
- Driver practice sessions
- Field-centric driving training
- Multi-speed driving practice
- Situations requiring precision and speed flexibility
- When dedicated odometry wheels aren't available

✅ **Recommended Configurations:**
- **Beginner drivers**: Start with MEDIUM preset, slow mode available
- **Experienced drivers**: FAST preset, field-centric enabled
- **Precision tasks**: Use DPAD controls for fine positioning
- **Endgame**: Driver 2 takes over at 50% power for careful maneuvers

---

## Technical Specifications

### Common Constants

```java
JOYSTICK_DEADZONE = 0.1 (10%)
SLOW_MODE_MULTIPLIER = 0.3 (30%)
```

### IMU Configuration

```java
Logo Facing: LEFT
USB Facing: FORWARD
Angle Unit: RADIANS (internal), DEGREES (display)
```

### Motor Configuration

```java
Direction: All motors set to FORWARD
Run Mode: RUN_WITHOUT_ENCODER (power control)
Zero Power Behavior: BRAKE
```

### Mecanum Drive Kinematics

```java
frontLeftPower  = -y + x + rx
backLeftPower   = -y - x + rx
frontRightPower = -y - x - rx
backRightPower  = -y + x - rx

Where:
  x  = forward/backward input
  y  = strafe input
  rx = rotation input
```

---

## Configuration Guide

### Step 1: Hardware Setup

1. **Mount motors** in mecanum configuration (X-pattern wheels)
2. **Connect encoders** to motor ports
3. **Install IMU** on robot chassis (logo facing left, USB forward)
4. **For ChassisTest**: Install dedicated odometry wheels at specified offsets

### Step 2: Configure Robot Controller

1. Open FTC Robot Controller app
2. Navigate to "Configure Robot"
3. Add devices matching the names in code:
   - Motors: `frontLeftMotor`, `backLeftMotor`, `frontRightMotor`, `backRightMotor`
   - IMU: `imu`
   - ChassisTest only: `xOdo`, `yOdo`

### Step 3: Verify Motor Directions

1. Deploy **ChassisTest**
2. Use **DPAD DOWN** to cycle to individual motor tests
3. Push **left stick forward** for each motor
4. Verify wheel spins in correct direction:
   - Front left should spin to drive robot forward
   - Back left should spin to drive robot forward
   - Front right should spin to drive robot forward
   - Back right should spin to drive robot forward
5. If any wheel spins backward, update `setDirection()` in code

### Step 4: Calibrate Odometry (ChassisTest only)

1. Place robot on field with clear reference
2. Press BACK to reset position
3. Drive forward 24 inches using tape measure
4. Check telemetry reported distance
5. Adjust `WHEEL_DIAMETER_MM` if needed: `actual_diameter = reported_distance / 24 * current_diameter`

### Step 5: Test IMU Heading

1. Press LEFT BUMPER (JustMotion) or BACK (ChassisTest) to reset heading
2. Rotate robot 90° clockwise
3. Verify heading shows approximately -90°
4. If incorrect, check IMU mounting orientation

---

## Troubleshooting

### Robot Drives in Wrong Direction

**Symptom**: Commands result in unexpected movement direction

**Solutions**:
1. Check motor directions in configuration
2. Use ChassisTest individual motor tests
3. Verify mecanum wheel orientation (X-pattern)
4. Adjust `setDirection()` for specific motors

### Odometry Drift

**Symptom**: Position tracking becomes inaccurate over time

**ChassisTest Solutions**:
1. Check odometry wheel contact with ground
2. Verify encoder connections
3. Recalibrate wheel diameter
4. Ensure wheels spin freely without binding

**JustMotion Solutions**:
1. Check for wheel slip during aggressive driving
2. Verify all four motor encoders are working
3. Adjust `COUNTS_PER_MOTOR_REV` if using different motors
4. Consider switching to ChassisTest with dedicated wheels

### IMU Heading Incorrect

**Symptom**: Heading doesn't match robot orientation

**Solutions**:
1. Verify IMU mounting: Logo facing LEFT, USB facing FORWARD
2. Update `IMU.Parameters` if mounting differs
3. Reset heading with LEFT BUMPER (JustMotion) or BACK (ChassisTest)
4. Check for magnetic interference near IMU

### Slow Response or Lag

**Symptom**: Robot responds slowly to controls

**Solutions**:
1. Check battery voltage (should be >12V)
2. Verify no loose connections
3. Reduce telemetry update frequency if needed
4. Check for motor stalling or binding

### Robot Won't Move

**Symptom**: Motors don't respond to controls

**Solutions**:
1. Check gamepad connection
2. Verify motor power limits aren't too low
3. Press A button (ChassisTest) to release emergency stop
4. Check joystick past deadzone threshold
5. Verify motors configured in Robot Controller app

### Field-Centric Mode Not Working

**Symptom**: Field-centric feels like robot-centric

**Solutions**:
1. Reset heading with LEFT BUMPER before enabling field-centric
2. Ensure IMU is working (check heading display)
3. Verify heading updates in telemetry while rotating
4. Robot must be facing "forward" when heading reset

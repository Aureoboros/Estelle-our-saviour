# FTC Robot TeleOp - Complete Control Guide

## 📋 Table of Contents
1. [Chassis Test Mode](#chassis-test-mode)
2. [Full Robot TeleOp](#full-robot-teleop)
3. [Quick Reference Cards](#quick-reference-cards)

---

# 🔧 Chassis Test Mode

**TeleOp Name**: `Chassis Test ONLY`

Use this mode to test and diagnose the drivetrain before running the full robot code.

## Test Mode Selection (DPAD)

| Button | Test Mode | What It Does |
|--------|-----------|--------------|
| **DPAD UP** | Normal Drive | Full mecanum drive with all 4 motors working together |
| **DPAD DOWN** | Individual Motors | Cycles through testing one motor at a time (FL → BL → FR → BR) |
| **DPAD LEFT** | Motor Pairs (Front/Back) | Tests front pair (FL+FR) or back pair (BL+BR) together |
| **DPAD RIGHT** | Motor Sides (Left/Right) | Tests left side (FL+BL) or right side (FR+BR) together |

## Control Inputs

### Normal Drive Mode (DPAD UP selected)
| Control | Function |
|---------|----------|
| **Left Stick Y** | Forward/Backward |
| **Right Stick X** | Strafe Left/Right |
| **Left Trigger (LT)** | Rotate Left |
| **Right Trigger (RT)** | Rotate Right |
| **START** | Toggle Slow Mode (30%) |

### Test Modes (DPAD DOWN/LEFT/RIGHT selected)
| Control | Function |
|---------|----------|
| **Left Stick Y** | Control active motor(s) forward/backward |
| **DPAD DOWN** | Cycle to next individual motor |
| **DPAD LEFT** | Switch between front/back pairs |
| **DPAD RIGHT** | Switch between left/right sides |

### Universal Controls (All Modes)
| Button | Function |
|--------|----------|
| **A** | Emergency Stop (all motors to 0) |
| **BACK** | Reset IMU and Odometry to zero |

## Telemetry Display

The driver station shows:
- **Current Test Mode**: Which motors are being tested
- **Active Motors**: Marked with arrows (◄◄◄) next to their power values
- **Motor Powers**: All 4 drive motors (FL, BL, FR, BR)
- **Encoder Counts**: 
  - Drive motors: FL, BL, FR, BR encoders
  - Odometry wheels: X Odo, Y Odo
- **Position (Odometry)**:
  - X Position (inches)
  - Y Position (inches)
  - Heading (degrees)

## Testing Checklist

### Step 1: Individual Motor Test
1. Press **DPAD DOWN** to enter individual motor mode
2. Use **Left Stick Y** to test Front Left motor
3. Verify motor spins in correct direction
4. Press **DPAD DOWN** again to cycle to Back Left
5. Repeat for all 4 motors
6. **Expected**: Each motor spins independently in correct direction

### Step 2: Motor Pairs Test
1. Press **DPAD LEFT** to test Front Pair
2. Both front motors should spin together
3. Press **DPAD LEFT** again to test Back Pair
4. Both back motors should spin together
5. **Expected**: Pairs spin synchronously

### Step 3: Side Test
1. Press **DPAD RIGHT** to test Left Side
2. FL and BL should spin together
3. Press **DPAD RIGHT** again to test Right Side
4. FR and BR should spin together
5. **Expected**: Sides spin synchronously

### Step 4: Full Drive Test
1. Press **DPAD UP** for normal drive mode
2. Test forward/backward movement (Left Stick Y)
3. Test strafing left/right (Right Stick X)
4. Test rotation (LT/RT triggers)
5. **Expected**: Robot moves smoothly in all directions

### Step 5: Odometry Verification
1. Press **BACK** to reset position to (0, 0)
2. Drive forward 24 inches
3. Check telemetry - Y Position should show ~24 inches
4. Strafe right 24 inches
5. Check telemetry - X Position should show ~24 inches
6. **Expected**: Odometry tracks position accurately (±2 inches)

## Common Issues & Solutions

| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| Motor doesn't spin | Wiring disconnected | Check motor port connection |
| Motor spins backwards | Motor direction reversed | Flip motor direction in code |
| Encoder count not changing | Encoder cable loose | Check encoder connection |
| Robot drifts during forward | Motors unbalanced | Check motor power equality |
| Odometry position wrong | Wheel diameter incorrect | Verify 32mm deadwheel size |
| Robot won't move | Emergency stop active | Press A button to clear |

---

# 🎮 Full Robot TeleOp

**TeleOp Name**: `New Robot Teleop v2`

Complete robot control with all mechanisms (intake, launcher, turret, spindexer).

## Control Layout

### Button Controls (Both Gamepads)
All buttons use **rising edge detection** - press once for one action.

| Button | Function | Details |
|--------|----------|---------|
| **B** | Intake Toggle | Starts/stops intake motor. Automatically rotates spindexer servo 120° twice (300ms delay between) |
| **X** | Aim & Shoot | Toggles launch sequence. Spins up launcher and activates turret auto-aim to alliance AprilTag. Press again to stop. |
| **Y** | Reverse Intake | Reverses intake motor direction (only works when intake is active) |
| **A** | Emergency Stop | Immediately stops ALL mechanisms (intake, launcher, turret) |
| **START** | Slow Mode Toggle | Reduces drive speed to 30% for precise movements |
| **BACK** | Reset Position | Resets IMU heading and odometry position to (0, 0) |
| **DPAD UP** | Select Blue Alliance | Sets target AprilTag to ID 20 (Blue) |
| **DPAD DOWN** | Select Red Alliance | Sets target AprilTag to ID 24 (Red) |
| **DPAD LEFT** | Decrease Launch Power | Reduces launcher speed by 5%. Wraps from 50% to 100% |
| **DPAD RIGHT** | Increase Launch Power | Increases launcher speed by 5%. Wraps from 100% to 50% |
| **Left Bumper** | *(Available)* | Not currently assigned |
| **Right Bumper** | *(Available)* | Not currently assigned |

### Joystick & Trigger Controls

| Control | Function | Details |
|---------|----------|---------|
| **Left Stick Y** | Forward/Backward | Push up to move forward, down to move backward |
| **Right Stick X** | Strafe Left/Right | Push left to strafe left, right to strafe right |
| **Left Trigger (LT)** | Rotate Left | Proportional rotation counter-clockwise |
| **Right Trigger (RT)** | Rotate Right | Proportional rotation clockwise |
| **Right Stick Y** | *(Available)* | Not currently assigned |

---

## 🤖 Robot Mechanisms

### **Alliance Selection** ⚠️ **Set This First!**
- **DPAD UP**: Select **Blue Alliance** (targets AprilTag 20)
- **DPAD DOWN**: Select **Red Alliance** (targets AprilTag 24)
- Alliance selection determines which AprilTag the turret aims at
- Default: Blue (Tag 20) if none selected

### **Launch Power Adjustment**
- **DPAD RIGHT**: Increase launch power by 5%
- **DPAD LEFT**: Decrease launch power by 5%
- **Range**: 50% to 100% (adjustable in 5% increments)
- **Wrap-around**: Pressing RIGHT at 100% → 50%, pressing LEFT at 50% → 100%
- **Default**: 75% power
- Changes apply immediately if launcher is running

### **Intake System**
- **Motor**: `intakeMotor` - runs at 100% power
- **Spindexer Servo**: Automatically indexes 120° twice during intake activation
  - Position 1: 0.0 (0°)
  - Position 2: 0.33 (120°)
  - Position 3: 0.67 (240°)
  - Delay: 300ms between rotations

### **Launch System**
- **Motor**: `launchMotor` - runs at adjustable power (50%-100%)
- **Default Power**: 75%
- **Turret Servo**: Continuous rotation servo for auto-aiming
  - Auto-aims to alliance-specific AprilTag (Red: 24, Blue: 20)
  - Searches 360° if tag not found (3 second timeout)
  - Returns to center (forward) if tag not detected
  - Aiming tolerance: ±3°

### **Drive System**
- **Type**: Mecanum drive (holonomic)
- **Max Speed**: 50% power (50% for both Gamepad 1 & 2)
- **Slow Mode**: 30% of normal speed (15% max power)
- **Odometry**: Real-time position tracking using two 32mm deadwheels (2000 CPR)

---

## 📊 Odometry & Position Tracking

The robot continuously tracks its position on the field using:
- **X Odometry Wheel** (`xOdo`): Tracks left/right movement
- **Y Odometry Wheel** (`yOdo`): Tracks forward/backward movement
- **IMU**: Tracks heading (rotation angle)

**Position Data:**
- X Position (inches) - left/right displacement from start
- Y Position (inches) - forward/backward displacement from start
- Heading (degrees) - rotation angle from start orientation

**Reset Position**: Press BACK button to reset all to zero

---

## 🎯 Auto-Aim Feature

### **Alliance Setup (REQUIRED)**
Before starting, select your alliance:
- **DPAD UP** = Blue Alliance → targets AprilTag 20
- **DPAD DOWN** = Red Alliance → targets AprilTag 24

### **Launch Sequence**
When **X button** is pressed to activate launch mode:

1. **Launcher spins up** at current power setting (50-100%)
2. **Turret begins searching** for alliance-specific AprilTag
3. **If tag detected**:
   - Turret aims proportionally (corrects yaw error)
   - Locks on when within ±3° of target
   - Status shows "LOCKED" with angle offset
4. **If tag NOT detected**:
   - Turret rotates 360° searching for tag
   - After 3 seconds, returns to forward position
   - Status shows "SEARCHING..." then "TARGET LOST"

**Targets**:
- **Red Alliance**: AprilTag ID 24
- **Blue Alliance**: AprilTag ID 20 (default)

---

## 🔧 Technical Constants

### Drive Configuration
```java
GAMEPAD1_MAX_POWER = 0.5  // 50% max speed
GAMEPAD2_MAX_POWER = 0.5  // 50% max speed
SLOW_MODE_MULTIPLIER = 0.3  // 30% in slow mode
JOYSTICK_DEADZONE = 0.1  // 10% deadzone
```

### Odometry Configuration
```java
WHEEL_DIAMETER = 32mm (1.26 inches)
COUNTS_PER_REV = 2000
COUNTS_PER_INCH = 499.29
```

### Servo Configuration
```java
SPINDEXER_DELAY = 300ms
TURRET_SEARCH_SPEED = 0.3
TURRET_AIM_SPEED = 0.15
TURRET_SEARCH_TIMEOUT = 3.0 seconds
ANGLE_TOLERANCE = ±3°
```

### Launch Motor Configuration
```java
LAUNCH_MOTOR_MIN = 0.5  // 50% minimum
LAUNCH_MOTOR_MAX = 1.0  // 100% maximum
LAUNCH_MOTOR_STEP = 0.05  // 5% per button press
DEFAULT_POWER = 0.75  // 75% starting power
```

### AprilTag Configuration
```java
RED_APRILTAG_ID = 24
BLUE_APRILTAG_ID = 20
```

---

## 🚨 Important Notes

### Driver Priority
- **Gamepad 1 has priority** over Gamepad 2
- If both gamepads give input, Gamepad 1 controls are used
- Both gamepads can activate mechanisms (buttons)

### Rising Edge Detection
- ✅ **ALL buttons and DPAD directions** use rising edge detection
- One button press = one action (no repeating)
- Toggle states preserved (intake/launch stay active until toggled off)
- DPAD adjustments increment/decrement by one step per press

### Safety Features
- **A button** emergency stops ALL mechanisms instantly
- **Brake mode** enabled on all motors (stops quickly when no input)
- **Power limits** prevent motor damage (max 50% drive power)

---

## 📱 Telemetry Display

The driver station shows:
- **Alliance**: Current alliance selection (RED/BLUE/NONE)
- **Target AprilTag**: Which tag ID will be targeted (24/20)
- **Active Driver**: Which gamepad is controlling the robot
- **Slow Mode**: ON/OFF status
- **Position**: X, Y coordinates and heading angle
- **Mechanisms**: Status of intake, launcher, spindexer position
- **Launch Power**: Current launcher power percentage with adjustment reminder
- **Turret**: LOCKED/SEARCHING/IDLE with angle information
- **Motor Powers**: All 4 drive motor power levels (FL, BL, FR, BR)

---

## 📋 Pre-Match Checklist

### Before Every Match:
1. ✅ **Run Chassis Test** to verify all motors working
2. ✅ **Select alliance** (DPAD UP for Blue, DPAD DOWN for Red)
3. ✅ **Adjust launch power** if needed (DPAD LEFT/RIGHT, default 75%)
4. ✅ **Verify camera** can see AprilTags
5. ✅ **Reset position** (BACK button) at starting position
6. ✅ **Test intake and launcher** before match starts (B and X buttons)
7. ✅ **Check odometry** is tracking correctly (drive and watch telemetry)

### Troubleshooting During Match:
- Robot not moving? → Press **A** to clear emergency stop
- Wrong AprilTag targeted? → Press **DPAD UP** or **DOWN** to select correct alliance
- Launcher too weak/strong? → Press **DPAD LEFT/RIGHT** to adjust (live adjustment)
- Turret can't find tag? → Position robot to see AprilTag clearly
- Position tracking off? → Press **BACK** to reset odometry

---

## ⚙️ Configuration

To change settings, modify these constants in the code:
- `RED_APRILTAG_ID` / `BLUE_APRILTAG_ID` - Alliance target tags (default: 24/20)
- `LAUNCH_MOTOR_MIN` / `LAUNCH_MOTOR_MAX` - Power range (default: 50%-100%)
- `LAUNCH_MOTOR_STEP` - Adjustment increment (default: 5%)
- `TURRET_SEARCH_TIMEOUT` - How long to search (default: 3.0s)
- `GAMEPAD1_MAX_POWER` / `GAMEPAD2_MAX_POWER` - Max drive speed

---

# 🎮 Quick Reference Cards

## Chassis Test Quick Reference
```
╔═══════════════════════════════════════════════════════════════╗
║                    CHASSIS TEST CONTROLS                      ║
╠═══════════════════════════════════════════════════════════════╣
║  MODE SELECTION:                                              ║
║    DPAD ↑ = Normal Drive         DPAD ↓ = Individual Motors   ║
║    DPAD ← = Motor Pairs          DPAD → = Sides               ║
║                                                               ║
║  NORMAL DRIVE:                                                ║
║    Left Stick Y = Forward/Back   Right Stick X = Strafe L/R   ║
║    LT/RT = Rotate Left/Right                                  ║
║                                                               ║
║  TEST MODE:                                                   ║
║    Left Stick Y = Control Active Motor(s)                     ║
║                                                               ║
║  UNIVERSAL:                                                   ║
║    A = Emergency Stop            BACK = Reset Position        ║
║    START = Slow Mode Toggle                                   ║
╚═══════════════════════════════════════════════════════════════╝
```

## Full Robot Quick Reference
```
╔═══════════════════════════════════════════════════════════════╗
║                    FTC TELEOP CONTROLS                        ║
╠═══════════════════════════════════════════════════════════════╣
║  ALLIANCE:    DPAD ↑ = Blue (Tag 20)  |  DPAD ↓ = Red (Tag 24)║
║  LAUNCHER:    DPAD → = +5% Power      |  DPAD ← = -5% Power   ║
║                                                               ║
║  B = Intake Toggle      |  X = Aim & Shoot                    ║
║  Y = Reverse Intake     |  A = EMERGENCY STOP                 ║
║                                                               ║
║  START = Slow Mode      |  BACK = Reset Position              ║
║                                                               ║
║  Left Stick Y = Forward/Back  |  Right Stick X = Strafe L/R   ║
║  LT/RT = Rotate Left/Right                                    ║
╚═══════════════════════════════════════════════════════════════╝
```

## Competition Day Workflow
```
1. Power on robot
2. Run "Chassis Test ONLY" 
   → Verify all 4 drive motors working
   → Check odometry encoders counting
3. Switch to "New Robot Teleop v2"
4. Select alliance (DPAD UP/DOWN)
5. Press BACK at starting position
6. Ready for match!
```

---

**Version**: Phase 1 Complete  
**Status**: ✅ Rising Edge Detection on ALL Buttons + DPAD  
**Status**: ✅ Chassis Test Mode Available  
**Ready for**: Phase 2 (Limelight & AprilTag Navigation)  

**Last Updated**: December 2024

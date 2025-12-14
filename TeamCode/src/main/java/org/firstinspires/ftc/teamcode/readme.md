# FTC Robot TeleOp - Complete Control Guide

##  Table of Contents
1. [Chassis Test Mode](#chassis-test-mode)
2. [Phase 1 - Full Robot TeleOp](#phase-1---full-robot-teleop)
3. [Phase 2 - Limelight Fusion](#phase-2---limelight-fusion)
4. [Quick Reference Cards](#quick-reference-cards)

---

#  Chassis Test Mode

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
  - Odometry wheels: X Odo, Y Odo (if equipped)
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

### Step 5: Odometry Verification (if equipped)
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
| Odometry position wrong | Wheel diameter incorrect | Verify deadwheel size |
| Robot won't move | Emergency stop active | Press A button to clear |

---

#  Phase 1 - Full Robot TeleOp

**TeleOp Name**: `Full Robot Teleop v2`

Complete robot control with all mechanisms (intake, launcher, turret, spindexer, armor).

## Control Layout

### Button Controls (Both Gamepads)
All buttons use **rising edge detection** - press once for one action.

| Button | Function | Details |
|--------|----------|---------|
| **B** | Intake Toggle | Starts/stops intake motor. Automatically rotates spindexer servo 120° twice (300ms delay between) |
| **X** | Aim & Shoot | Toggles launch sequence. Spins up launcher and activates turret auto-aim to alliance AprilTag. Press again to stop. |
| **Y** | Reverse Intake | Reverses intake motor direction (only works when intake is active) |
| **A** | Emergency Stop | Immediately stops ALL mechanisms (intake, launcher, turret, armor) |
| **LEFT BUMPER (LB)** | Armor Toggle  | Deploy/retract armor plates for stable shooting position |
| **RIGHT BUMPER (RB)** | *(Available)* | Reserved for Phase 2 |
| **START** | Slow Mode Toggle | Reduces drive speed to 30% for precise movements |
| **BACK** | Reset Position | Resets IMU heading and odometry position to (0, 0) |
| **DPAD UP** | Select Blue Alliance | Sets target AprilTag to ID 20 (Blue) |
| **DPAD DOWN** | Select Red Alliance | Sets target AprilTag to ID 24 (Red) |
| **DPAD LEFT** | Decrease Launch Power | Reduces launcher speed by 5%. Wraps from 50% to 100% |
| **DPAD RIGHT** | Increase Launch Power | Increases launcher speed by 5%. Wraps from 100% to 50% |

### Joystick & Trigger Controls

| Control | Function | Details |
|---------|----------|---------|
| **Left Stick Y** | Forward/Backward | Push up to move forward, down to move backward |
| **Right Stick X** | Strafe Left/Right | Push left to strafe left, right to strafe right |
| **Left Trigger (LT)** | Rotate Left | Proportional rotation counter-clockwise |
| **Right Trigger (RT)** | Rotate Right | Proportional rotation clockwise |

---

## 🤖 Robot Mechanisms (Phase 1)

### **Alliance Selection**  **Set This First!**
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

### **Armor System** 
- **LEFT BUMPER (LB)**: Toggle armor deployment
- **Deploy**: Motors continuously push plates down to ground (40% power)
- **Retract**: Strong pulse (80% reverse) for 500ms, then stop
- **Purpose**: Create stable base when shooting to resist being pushed
- Use before shooting for maximum stability

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
- **Max Speed**: 50% power (for both Gamepad 1 & 2)
- **Slow Mode**: 30% of normal speed (15% max power)
- **Odometry**: Real-time position tracking using drive motor encoders

---

##  Position Tracking (Phase 1)

The robot continuously tracks its position using:
- **Drive Motor Encoders**: Tracks all movement (4-wheel odometry)
- **IMU**: Tracks heading (rotation angle)

**Position Data:**
- X Position (inches) - left/right displacement from start
- Y Position (inches) - forward/backward displacement from start
- Heading (degrees) - rotation angle from start orientation

**Accuracy**: ±2-4 inches over short distances, accumulates drift over time

**Reset Position**: Press BACK button to reset all to zero

---

##  Auto-Aim Feature (Phase 1)

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

#  Phase 2 - Limelight Fusion

**TeleOp Name**: `Phase 2 - Limelight Fusion`

Advanced positioning with triple-source fusion and auto-navigation.

## New Features in Phase 2

### **Triple-Source Position Fusion** 
Phase 2 intelligently combines three positioning sources:

1. **Drive Encoder Odometry** (continuous baseline)
   - Tracks all robot movement
   - Updates every loop (~50Hz)
   - Accumulates drift over time

2. **Limelight Vision** (fast correction)
   - Absolute position from vision target
   - Fast updates (30-60Hz)
   - Corrects odometry drift automatically

3. **AprilTag Vision** (verification)
   - High-accuracy absolute position
   - Cross-checks Limelight data
   - Backup when Limelight unavailable

### **Intelligent Fusion Algorithm**
The system automatically:
- ✅ Detects drift between odometry and vision
- ✅ Corrects position with vision data
- ✅ Tracks position confidence (0-100%)
- ✅ Detects when robot is pushed
- ✅ Cross-validates vision sources
- ✅ Warns when position unreliable

---

## Control Layout (Phase 2)

### **NEW Controls**

| Button | Function | Details |
|--------|----------|---------|
| **RIGHT BUMPER (RB)** | Auto-Navigate | Automatically drives to scoring position near alliance AprilTag. Press again or use joystick to cancel. |

### **All Other Controls**: Same as Phase 1
- **B** = Intake toggle
- **X** = Aim & shoot
- **Y** = Reverse intake
- **A** = Emergency stop (also cancels auto-nav)
- **LB** = Armor toggle
- **START** = Slow mode toggle
- **BACK** = Reset all positioning
- **DPAD UP/DOWN** = Alliance selection
- **DPAD LEFT/RIGHT** = Launch power adjustment
- **Left Stick Y** = Forward/backward (overrides auto-nav)
- **Right Stick X** = Strafe left/right (overrides auto-nav)
- **LT/RT** = Rotate left/right (overrides auto-nav)

---

##  Position Fusion System

### **How It Works**

```
┌─────────────────────────────────────────────┐
│      POSITION ESTIMATION SYSTEM             │
├─────────────────────────────────────────────┤
│                                             │
│  1. Drive Encoders (Continuous)             │
│     └─ Always tracking, may drift           │
│                                             │
│  2. Limelight Vision (Fast Correction)      │
│     └─ Corrects drift when target visible   │
│                                             │
│  3. AprilTag (Verification)                 │
│     └─ Cross-checks Limelight accuracy      │
│                                             │
│  → FUSED POSITION (Best Estimate)           │
│     └─ Displayed in telemetry               │
└─────────────────────────────────────────────┘
```

### **Correction Modes**

| Drift Amount | Action | Description |
|--------------|--------|-------------|
| < 2 inches | Trust Odometry | Good agreement, keep current estimate |
| 2-6 inches | Blend | Mix 70% odometry + 30% vision |
| > 6 inches | Hard Reset | Large drift, reset to vision position |

### **Confidence Tracking**

- **Starts at**: 100% confidence
- **Decays**: 2% per loop without vision correction
- **Restored**: +10-15% when vision correction applied
- **Low confidence** (<50%): Position may be inaccurate

### **Push Detection**

The system detects when your robot is unexpectedly moved:
1. Monitors odometry vs vision when motors idle
2. If position changes >3 inches without motor input
3. Alerts " ROBOT WAS PUSHED!"
4. Auto-corrects position to vision

### **Vision Cross-Validation**

When both Limelight and AprilTag are visible:
- System compares their position estimates
- If they disagree by >8 inches:
  - Warns " VISION CONFLICT"
  - Trusts AprilTag (more accurate)
  - Resets odometry to AprilTag position

---

##  Auto-Navigation Feature

### **How to Use**

1. **Set Alliance** (DPAD UP/DOWN)
2. **Position robot** where you can see alliance AprilTag
3. **Press RIGHT BUMPER (RB)**
   - Robot calculates path to scoring position
   - Drives automatically to target
   - Status shows distance and target coordinates

4. **To Cancel**:
   - Press RB again, OR
   - Move joystick (driver override), OR
   - Press A (emergency stop)

### **Target Calculation**

The robot navigates to a scoring position:
- **12 inches** to the side of AprilTag
- **24 inches** in front of AprilTag
- Adjust these offsets in code based on your robot

### **Auto-Nav Behavior**

- **Speed**: Slows down as it approaches target
- **Tolerance**: Stops within 4 inches of target
- **Heading**: Rotates to face scoring direction
- **Override**: Any joystick input cancels auto-nav immediately

---

## 📱 Enhanced Telemetry (Phase 2)

### **Position Fusion Section**
- **Position**: Current X, Y coordinates
- **Heading**: Robot rotation angle
- **Source**: Which source is being used
  - `ODOMETRY` = Trusting drive encoders
  - `LIMELIGHT` = Using Limelight correction
  - `APRILTAG` = Using AprilTag correction
  - `FUSED` = Blending multiple sources
- **Confidence**: 0-100% position reliability
- ** Drift**: Warning if odometry drifts >3"

### **Vision Status Section**
- **Limelight**: 
  - "✓ Target Locked" with TX, TY coordinates
  - "No Target" if nothing visible
- **AprilTag**:
  - "✓ ID [#] visible" with tag number
  - "No Target" if nothing visible

### **Auto-Navigation Section** (when active)
- **Target**: Destination X, Y coordinates
- **Distance**: How far to target in inches
- **Note**: "(Driver input to cancel)"

### **All Other Sections**: Same as Phase 1
- Alliance, driver, slow mode
- Mechanisms (intake, launch, armor, turret, spindexer)
- Motor powers

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
COUNTS_PER_MOTOR_REV = 537.7  // REV HD Hex Motor
WHEEL_DIAMETER = 4.0 inches
COUNTS_PER_INCH = 42.79
```

### Position Fusion Configuration
```java
VISION_CORRECTION_LARGE = 6.0 inches  // Hard reset threshold
VISION_CORRECTION_SMALL = 2.0 inches  // Blend threshold
VISION_BLEND_RATIO = 0.3  // 30% vision, 70% odometry
CONFIDENCE_DECAY = 0.02  // 2% per loop
PUSH_DETECTION = 3.0 inches  // Unexpected movement
AUTO_NAV_SPEED = 0.25  // 25% max speed
AUTO_NAV_TOLERANCE = 4.0 inches  // Stop within 4"
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

### Armor Configuration
```java
ARMOR_DEPLOY_POWER = 0.4  // 40% continuous
ARMOR_RETRACT_POWER = -0.8  // 80% reverse pulse
ARMOR_RETRACT_TIME = 500ms
```

### AprilTag Configuration
```java
RED_APRILTAG_ID = 24
BLUE_APRILTAG_ID = 20
```

---

##  Important Notes

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
- **A button** emergency stops ALL mechanisms instantly (including auto-nav)
- **Brake mode** enabled on all motors (stops quickly when no input)
- **Power limits** prevent motor damage (max 50% drive power)
- **Driver override** - any joystick input cancels auto-navigation

### Position Accuracy (Phase 2)

| Scenario | Expected Accuracy | Primary Source |
|----------|------------------|----------------|
| Vision available, stationary | ±1-2 inches | Limelight/AprilTag |
| Vision available, moving | ±2-3 inches | Fused |
| No vision, short distance (<10 ft) | ±2-4 inches | Odometry |
| No vision, long distance (>20 ft) | ±6-12 inches | Odometry (drifted) |
| After vision correction | ±1-2 inches | Reset to vision |

---

## 📋 Pre-Match Checklist

### **Before Every Match:**

#### 1. Hardware Check
- ✅ Run **Chassis Test** to verify all motors working
- ✅ Check camera connections (Limelight + AprilTag webcam)
- ✅ Verify Limelight is connected (telemetry shows "✓ CONNECTED")
- ✅ Test armor deployment (LB button)

#### 2. Software Setup
- ✅ **Select alliance** (DPAD UP for Blue, DPAD DOWN for Red)
- ✅ **Adjust launch power** if needed (DPAD LEFT/RIGHT, default 75%)
- ✅ **Press BACK** at starting position to reset odometry to (0, 0)
- ✅ Verify position confidence is 100%

#### 3. Vision Verification
- ✅ Point robot at alliance AprilTag
- ✅ Verify telemetry shows "✓ Target Locked"
- ✅ Check that position updates when moved
- ✅ Confirm auto-aim works (press X near tag)

#### 4. Mechanism Test
- ✅ Test intake (B button) - should rotate spindexer
- ✅ Test launcher (X button) - should spin up and aim
- ✅ Test armor (LB button) - should deploy smoothly
- ✅ Test auto-nav (RB button) - should calculate target

#### 5. Final Verification
- ✅ Check telemetry shows correct alliance
- ✅ Verify slow mode toggle works (START)
- ✅ Test emergency stop (A button)
- ✅ Ready for match! 

---

### **Troubleshooting During Match:**

| Issue | Solution |
|-------|----------|
| Robot not moving | Press **A** to clear emergency stop |
| Wrong AprilTag targeted | Press **DPAD UP** or **DOWN** to select correct alliance |
| Launcher too weak/strong | Press **DPAD LEFT/RIGHT** to adjust (live adjustment) |
| Turret can't find tag | Position robot to see AprilTag clearly |
| Position tracking off | Press **BACK** to reset odometry |
| Low confidence warning | Drive near AprilTag to restore vision correction |
| "Vision Conflict" warning | System auto-correcting, continue normally |
| "Robot Was Pushed" alert | System auto-corrected position |
| Auto-nav not working | Ensure alliance selected and AprilTag visible |
| Armor won't retract | Press **A** (emergency stop), then LB again |

---

#  Quick Reference Cards

## Chassis Test Quick Reference
```
╔═══════════════════════════════════════════════════════════════╗
║                    CHASSIS TEST CONTROLS                       ║
╠═══════════════════════════════════════════════════════════════╣
║  MODE SELECTION:                                               ║
║    DPAD ↑ = Normal Drive         DPAD ↓ = Individual Motors   ║
║    DPAD ← = Motor Pairs          DPAD → = Sides               ║
║                                                                ║
║  NORMAL DRIVE:                                                 ║
║    Left Stick Y = Forward/Back   Right Stick X = Strafe L/R   ║
║    LT/RT = Rotate Left/Right                                  ║
║                                                                ║
║  TEST MODE:                                                    ║
║    Left Stick Y = Control Active Motor(s)                     ║
║                                                                ║
║  UNIVERSAL:                                                    ║
║    A = Emergency Stop            BACK = Reset Position        ║
║    START = Slow Mode Toggle                                   ║
╚═══════════════════════════════════════════════════════════════╝
```

## Phase 1 Quick Reference
```
╔═══════════════════════════════════════════════════════════════╗
║                  PHASE 1 - FULL ROBOT CONTROLS                 ║
╠═══════════════════════════════════════════════════════════════╣
║  ALLIANCE:    DPAD ↑ = Blue (Tag 20)  |  DPAD ↓ = Red (Tag 24)║
║  LAUNCHER:    DPAD → = +5% Power      |  DPAD ← = -5% Power   ║
║                                                                ║
║  B = Intake Toggle      |  X = Aim & Shoot                    ║
║  Y = Reverse Intake     |  A = EMERGENCY STOP                 ║
║  LB = Armor Toggle     |  (Deploy/Retract stabilizers)       ║
║                                                                ║
║  START = Slow Mode      |  BACK = Reset Position              ║
║                                                                ║
║  Left Stick Y = Forward/Back  |  Right Stick X = Strafe L/R   ║
║  LT/RT = Rotate Left/Right                                    ║
╚═══════════════════════════════════════════════════════════════╝
```

## Phase 2 Quick Reference
```
╔═══════════════════════════════════════════════════════════════╗
║              PHASE 2 - LIMELIGHT FUSION CONTROLS               ║
╠═══════════════════════════════════════════════════════════════╣
║  ALLIANCE:    DPAD ↑ = Blue (Tag 20)  |  DPAD ↓ = Red (Tag 24)║
║  LAUNCHER:    DPAD → = +5% Power      |  DPAD ← = -5% Power   ║
║                                                                ║
║  B = Intake Toggle      |  X = Aim & Shoot                    ║
║  Y = Reverse Intake     |  A = EMERGENCY STOP                 ║
║  LB = Armor Toggle    |  RB = Auto-Navigate                  ║
║                                                                ║
║  START = Slow Mode      |  BACK = Reset Position              ║
║                                                                ║
║  Left Stick Y = Forward/Back  |  Right Stick X = Strafe L/R   ║
║  LT/RT = Rotate Left/Right                                    ║
║                                                                ║
║  POSITION FUSION: Odometry + Limelight + AprilTag             ║
║  • Watch confidence % in telemetry                            ║
║  • Vision auto-corrects drift                                 ║
║  • Auto-nav drives to scoring position                        ║
╚═══════════════════════════════════════════════════════════════╝
```

## Competition Day Workflow
```
┌─────────────────────────────────────────────┐
│        COMPETITION DAY WORKFLOW             │
├─────────────────────────────────────────────┤
│                                             │
│  1. Power on robot                          │
│                                             │
│  2. Run "Chassis Test ONLY"                 │
│     → Verify all 4 drive motors working    │
│     → Check odometry encoders counting      │
│                                             │
│  3. Switch to "Full Robot Teleop v2"        │
│     (or "Phase 2 - Limelight Fusion")      │
│                                             │
│  4. Select alliance (DPAD UP/DOWN)          │
│                                             │
│  5. Test all mechanisms:                    │
│     → Intake (B)                            │
│     → Launcher (X)                          │
│     → Armor (LB)                            │
│     → Vision targeting                      │
│                                             │
│  6. Press BACK at starting position         │
│                                             │
│  7. Ready for match!                        │
│                                             │
└─────────────────────────────────────────────┘
```

---

##  Feature Comparison

| Feature | Chassis Test | Phase 1 | Phase 2 |
|---------|--------------|---------|---------|
| **Drive Control** | ✓ | ✓ | ✓ |
| **Individual Motor Test** | ✓ | ✗ | ✗ |
| **Intake System** | ✗ | ✓ | ✓ |
| **Launcher** | ✗ | ✓ | ✓ |
| **Turret Auto-Aim** | ✗ | ✓ | ✓ |
| **Armor Deployment** | ✗ | ✓ | ✓ |
| **Position Tracking** | Basic | Drive Encoders | Triple Fusion |
| **Vision Correction** | ✗ | ✗ | ✓ |
| **Limelight Integration** | ✗ | ✗ | ✓ |
| **Auto-Navigation** | ✗ | ✗ | ✓ |
| **Position Confidence** | ✗ | ✗ | ✓ |
| **Push Detection** | ✗ | ✗ | ✓ |
| **Vision Cross-Check** | ✗ | ✗ | ✓ |

---

**Version**: Phase 2 Complete  
**Status**: ✅ All Features Implemented  
**Hardware**: 8/8 Motor Ports Used, 2/6 Servo Ports Used  
**Last Updated**: December 2024

---

##  Need Help?

- **Motor not working?** → Run Chassis Test to diagnose
- **Vision not working?** → Check camera connections in telemetry
- **Position drifting?** → Phase 2 auto-corrects with vision
- **Robot behaving oddly?** → Press A for emergency stop, then BACK to reset

Good luck at competition! 🏆🤖

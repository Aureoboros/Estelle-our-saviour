package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOpTourneyMega")
public class TeleOpTourneySupreme extends LinearOpMode {

    // Motor power constants
    private static final double INTAKE_POWER = -1.0;
    private static double LAUNCH_MOTOR_POWER = -0.8;
    private static final double SPATULA_SERVO_POWER = 0.8;
    private static final double SPIN_SPIN_SERVO_POWER = 0.5;
    private static final double SPIN_SERVO_SPEED = 0.1;

    // Navigation constants
    private static final double AUTO_MAX_SPEED = 0.7;
    private static final double AUTO_MIN_SPEED = 0.15;
    private static final double SLOWDOWN_DISTANCE_FEET = 2.0;
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.5;

    // ========== LIMELIGHT MOUNTING SPECS ==========
    private static final double LIMELIGHT_HEIGHT_MM = 330.2;  // 13 inches off ground
    private static final double LIMELIGHT_ANGLE_DEG = 0.0;    // Mounted horizontal
    private static final double LIMELIGHT_ANGLE_RAD = Math.toRadians(LIMELIGHT_ANGLE_DEG);

    // Launch constants
    private static final double LAUNCH_HEIGHT_MM = 304.143;
    private static final double TARGET_HEIGHT_MM = 1143.0;  // 45 inches above ground (0.5ft above tag)
    private static final double TARGET_HEIGHT_ABOVE_TAG_MM = 152.4; // 6 inches above tag
    private static final double LAUNCH_ANGLE_DEG = 65.0;  // Updated to 65° as specified
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);
    private static final double GRAVITY_MM_S2 = 9810.0;
    private static final double MAX_MOTOR_RPM = 6000.0;
    private static final double FIELD_SIZE_MM = 3657.6; // 12 feet in mm
    private static final double WHEEL_DIAMETER_MM = 96.0; // Launch wheel diameter
    private static final int TARGET_APRILTAG_ID_RED = 24;  // Red alliance goal
    private static final int TARGET_APRILTAG_ID_BLUE = 20; // Blue alliance goal

    // DECODE Season AprilTag positions (in mm from field center)
    private static final double[][] TAG_POSITIONS = {
            {0.0, FIELD_SIZE_MM / 2},      // Tag 20 - Blue Alliance Goal
            {0.0, 1828.8},                 // Tag 21 - Neutral (0, 6ft)
            {0.0, 1828.8},                 // Tag 22 - Neutral (0, 6ft)
            {0.0, 1828.8},                 // Tag 23 - Neutral (0, 6ft)
            {0.0, -FIELD_SIZE_MM / 2}      // Tag 24 - Red Alliance Goal
    };

    // Field positions (in feet, measured from field center) - MIRRORED FOR RED
    private static final double RED_LOWER_SPIKE_X = 3.0;
    private static final double RED_LOWER_SPIKE_Y = -3.0;
    private static final double RED_MIDDLE_SPIKE_X = 3.0;
    private static final double RED_MIDDLE_SPIKE_Y = -1.0;
    private static final double RED_TOP_SPIKE_X = 3.0;
    private static final double RED_TOP_SPIKE_Y = 1.0;
    private static final double RED_SHOOT_X = 0.0;
    private static final double RED_SHOOT_Y = 4.0;
    private static final double OBELISK_X = 0.0;
    private static final double OBELISK_Y = 6.0;
    private static final double RED_DEFAULT_START_X = 1.0;
    private static final double RED_DEFAULT_START_Y = -5.0;
    private static final double FIELD_MIN_FEET = -6.0;
    private static final double FIELD_MAX_FEET = 6.0;

    // Odometry constants
    private static final double ODOMETRY_INCHES_PER_TICK = 0.01; // CALIBRATE THIS
    private static final double COUNTS_PER_MM = 1.0; // CALIBRATE THIS

    // Tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private boolean odometryInitialized = false;
    private int lastLeftEncoderPos = 0;
    private int lastRightEncoderPos = 0;
    private int lastStrafeEncoderPos = 0;
    private double DOWN = -0.1;
    private double UP = 0.1;
    private boolean positionDetected = false;

    private boolean targetingRed = true; // Default to RED alliance

    // ========== MOTOR POWER CONSTANTS ==========
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;

    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 1.0;
    private static final double GAMEPAD2_MAX_POWER = 0.5;

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor, launchMotor;
    private DcMotor xodo, yodo;
    private Servo spinSpinServo, spatulaServo, stopServo;
    private IMU imu;
    private Limelight3A limelight;

    // Speed presets
    private static final double SPEED_SLOW = 0.3;
    private static final double SPEED_MEDIUM = 0.6;
    private static final double SPEED_FAST = 1.0;

    // ========== DRIVE ENCODER ODOMETRY CONSTANTS ==========
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    private int lastFLEncoder = 0;
    private int lastBLEncoder = 0;
    private int lastFREncoder = 0;
    private int lastBREncoder = 0;
    private double currentPos;

    // Speed preset state
    private enum SpeedMode { SLOW, MEDIUM, FAST }
    private SpeedMode currentSpeedMode = SpeedMode.FAST;

    private double launchMotorPowerAdjust = SPEED_FAST;
    private int togglestopper = 1;
    private int toggleintake = 1;
    private int togglespatula = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        // Initialize gamepad state tracking
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Toggle states
        boolean slowMode = true;
        boolean fieldCentric = false;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // ========== UPDATE GAMEPAD STATES ==========
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // ========== UPDATE ODOMETRY ==========
            robotX = getXOdoInches() / 12;
            robotY = getYOdoInches() / 12;

            // ========== RISING EDGE DETECTION ==========
            boolean startPressed = (currentGamepad1.start && !previousGamepad1.start) ||
                    (currentGamepad2.start && !previousGamepad2.start);
            boolean backPressed = (currentGamepad1.back && !previousGamepad1.back) ||
                    (currentGamepad2.back && !previousGamepad2.back);
            boolean aPressed = (currentGamepad1.a && !previousGamepad1.a) ||
                    (currentGamepad2.a && !previousGamepad2.a);
            boolean bPressed = (currentGamepad1.b && !previousGamepad1.b) ||
                    (currentGamepad2.b && !previousGamepad2.b);
            boolean xPressed = (currentGamepad1.x && !previousGamepad1.x) ||
                    (currentGamepad2.x && !previousGamepad2.x);
            boolean yPressed = (currentGamepad1.y && !previousGamepad1.y) ||
                    (currentGamepad2.y && !previousGamepad2.y);
            boolean leftBumperPressed = (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) ||
                    (currentGamepad2.left_bumper && !previousGamepad2.left_bumper);
            boolean rightBumperPressed = (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) ||
                    (currentGamepad2.right_bumper && !previousGamepad2.right_bumper);

            // ========== AUTO-CALCULATE LAUNCH POWER FROM LIMELIGHT ==========
            calculateAndSetLaunchPower();
            launchMotor.setPower(-LAUNCH_MOTOR_POWER);

            // ========== TURRET CONTROL ==========
            double turretInput = 0;

            if(currentGamepad1.left_bumper) {
                turretInput += 1.0;
            }
            if(currentGamepad1.right_bumper) {
                turretInput -= 1.0;
            }
            if (currentGamepad2.left_bumper) {
                turretInput += 1.0;
            }
            if (currentGamepad2.right_bumper) {
                turretInput -= 1.0;
            }
            if (Math.abs(turretInput) > 0.1) {
                double currentServoPos = spinSpinServo.getPosition();
                double deltaPos = turretInput * SPIN_SERVO_SPEED * 0.02;
                double newPos = currentServoPos + deltaPos;
                newPos = Range.clip(newPos, 0.0, 0.2);
                spinSpinServo.setPosition(newPos);
            }

            // ========== D-PAD PIPELINE SELECTION FOR LIMELIGHT ==========
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                limelight.pipelineSwitch(1);  // Pipeline 1 for Tag 20 (Blue)
                targetingRed = false;
                telemetry.addLine("🔵 Switched to BLUE alliance (Tag 20, Pipeline 1)");
                telemetry.update();
                sleep(200);
            }
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                limelight.pipelineSwitch(0);  // Pipeline 0 for Tag 24 (Red)
                targetingRed = true;
                telemetry.addLine("🔴 Switched to RED alliance (Tag 24, Pipeline 0)");
                telemetry.update();
                sleep(200);
            }

            // ========== Y BUTTON - INTAKE TOGGLE ==========
            if (yPressed) {
                if (toggleintake == 1) {
                    intakeMotor.setPower(INTAKE_POWER);
                    toggleintake = 0;
                } else {
                    intakeMotor.setPower(0);
                    toggleintake = 1;
                }
            }

            // ========== A/B/X BUTTONS ==========
            if (aPressed) {
                driveToPositionOdoWheels(0, 0);
            }
            if (bPressed) {
                if (togglestopper == 1) {
                    stopServo.setPosition(0);
                    togglestopper = 0;
                } else {
                    stopServo.setPosition(1);
                    togglestopper = 1;
                }
            }
            if (xPressed) {
                launchBalls(1);
            }

            // ========== START BUTTON - RESET ODOMETRY ==========
            if (startPressed) {
                resetOdometryPods();
            }

            // ========== BACK BUTTON - TOGGLE ALLIANCE ==========
            if (currentGamepad2.back && !previousGamepad2.back) {
                targetingRed = !targetingRed;
                limelight.pipelineSwitch(targetingRed ? 0 : 1);
                String alliance = targetingRed ? "🔴 RED (Tag 24)" : "🔵 BLUE (Tag 20)";
                telemetry.addLine("Switched to " + alliance);
                telemetry.update();
                sleep(200);
            }

            // ========== DRIVE CONTROL ==========
            double y = 0, x = 0, rx = 0;
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";

            // Check for active gamepad
            boolean gamepad1Active = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad1.dpad_up || currentGamepad1.dpad_down;

            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.dpad_up || currentGamepad2.dpad_down;

            if (gamepad1Active) {
                y = currentGamepad1.left_stick_y;
                x = currentGamepad1.right_stick_x;
                rx = currentGamepad1.right_trigger - currentGamepad1.left_trigger;

                // Manual launch power adjustment with D-pad up/down
                if (currentGamepad1.dpad_up && !currentGamepad1.dpad_left && !currentGamepad1.dpad_right) {
                    LAUNCH_MOTOR_POWER = Range.clip(LAUNCH_MOTOR_POWER + UP, 0.0, 1.0);
                } else if (currentGamepad1.dpad_down && !currentGamepad1.dpad_left && !currentGamepad1.dpad_right) {
                    LAUNCH_MOTOR_POWER = Range.clip(LAUNCH_MOTOR_POWER + DOWN, 0.0, 1.0);
                }

                maxDrivePower = GAMEPAD1_MAX_POWER;
                activeDriver = "DRIVER 1";
            } else if (gamepad2Active) {
                y = currentGamepad2.left_stick_y;
                x = currentGamepad2.right_stick_x;
                rx = currentGamepad2.right_trigger - currentGamepad2.left_trigger;

                // DPAD precision control
                if (currentGamepad2.dpad_up) {
                    y = -0.3;
                    x = 0;
                    rx = 0;
                } else if (currentGamepad2.dpad_down) {
                    y = 0.3;
                    x = 0;
                    rx = 0;
                } else if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                    y = 0;
                    x = -0.3;
                    rx = 0;
                } else if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                    y = 0;
                    x = 0.3;
                    rx = 0;
                }

                maxDrivePower = GAMEPAD2_MAX_POWER;
                activeDriver = "DRIVER 2";
            }

            // Apply deadzone to joystick inputs
            x = applyDeadzone(x);
            y = applyDeadzone(y);
            rx = applyDeadzone(rx);

            // Apply speed preset
            double speedMultiplier = 1.0;
            switch (currentSpeedMode) {
                case SLOW:
                    speedMultiplier = SPEED_SLOW;
                    break;
                case MEDIUM:
                    speedMultiplier = SPEED_MEDIUM;
                    break;
                case FAST:
                    speedMultiplier = SPEED_FAST;
                    break;
            }

            if (slowMode) {
                speedMultiplier = SLOW_MODE_MULTIPLIER;
            }

            x *= speedMultiplier;
            y *= speedMultiplier;
            rx *= speedMultiplier;

            // ========== FIELD CENTRIC CONVERSION ==========
            if (fieldCentric) {
                double heading = robotHeading;
                double temp = y * Math.cos(-heading) - x * Math.sin(-heading);
                x = y * Math.sin(-heading) + x * Math.cos(-heading);
                y = temp;
            }

            // ========== MECANUM DRIVE CALCULATION ==========
            double frontLeftPower = -y + x + rx;
            double backLeftPower = -y - x + rx;
            double frontRightPower = -y - x - rx;
            double backRightPower = -y + x - rx;

            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > maxDrivePower) {
                double scale = maxDrivePower / maxPower;
                frontLeftPower *= scale;
                backLeftPower *= scale;
                frontRightPower *= scale;
                backRightPower *= scale;
            }

            frontLeftPower = Range.clip(frontLeftPower, -maxDrivePower, maxDrivePower);
            backLeftPower = Range.clip(backLeftPower, -maxDrivePower, maxDrivePower);
            frontRightPower = Range.clip(frontRightPower, -maxDrivePower, maxDrivePower);
            backRightPower = Range.clip(backRightPower, -maxDrivePower, maxDrivePower);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ========== TELEMETRY ==========
            telemetry.addLine("========== DRIVE ONLY MODE ==========");
            telemetry.addData("Active Driver", activeDriver);

            String speedModeStr = slowMode ? "SLOW MODE (30%)" :
                    currentSpeedMode == SpeedMode.SLOW ? "SLOW (30%)" :
                            currentSpeedMode == SpeedMode.MEDIUM ? "MEDIUM (60%)" :
                                    "FAST (100%)";
            telemetry.addData("Speed", speedModeStr);
            telemetry.addData("Control Mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addLine();

            telemetry.addLine("--- Limelight Targeting ---");
            String alliance = targetingRed ? "🔴 RED (Tag 24, Pipeline 0)" : "🔵 BLUE (Tag 20, Pipeline 1)";
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Launch Power", "%.2f (%.0f RPM)",
                    LAUNCH_MOTOR_POWER, LAUNCH_MOTOR_POWER * MAX_MOTOR_RPM);
            telemetry.addLine("D-PAD LEFT = Blue | D-PAD RIGHT = Red");
            telemetry.addLine();

            telemetry.addLine("--- Position (Odometry) ---");
            telemetry.addData("X Position", "%.2f ft", robotX);
            telemetry.addData("Y Position", "%.2f ft", robotY);
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(robotHeading));
            telemetry.addLine();

            telemetry.addLine("--- Drive Powers ---");
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BR", "%.2f", backRightPower);
            telemetry.addLine();

            telemetry.addData("Spin Servo", "%.2f", spinSpinServo.getPosition());
            telemetry.addLine();

            telemetry.addLine("--- Odometry Pods ---");
            telemetry.addData("xOdo Position", "%d ticks", getXOdoPosition());
            telemetry.addData("yOdo Position", "%d ticks", getYOdoPosition());
            telemetry.addData("xOdo Inches", "%.2f in", getXOdoInches());
            telemetry.addData("yOdo Inches", "%.2f in", getYOdoInches());

            telemetry.update();
        }
    }

    private void initializeHardware() {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        xodo = hardwareMap.get(DcMotor.class, "xOdo");
        yodo = hardwareMap.get(DcMotor.class, "yOdo");

        spinSpinServo = hardwareMap.get(Servo.class, "spinSpinServo");
        spatulaServo = hardwareMap.get(Servo.class, "spatulaServo");
        stopServo = hardwareMap.get(Servo.class, "stopServo");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // Start with Pipeline 0 (Red/Tag 24)
        limelight.start();

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopServo.setPosition(0.0);
        spatulaServo.setPosition(1.0);
        spinSpinServo.setPosition(0.01);

        telemetry.addLine("========================================");
        telemetry.addLine("TELEOP WITH LIMELIGHT AUTO-AIM");
        telemetry.addLine("========================================");
        telemetry.addData("Drive Motors", "✓");
        telemetry.addData("Limelight", "✓ (Pipeline 0 = Tag 24 Red)");
        telemetry.addData("Auto Launch Power", "✓ (Based on distance)");
        telemetry.addLine("========================================");
        telemetry.addLine("Controls:");
        telemetry.addLine("  D-PAD LEFT = Blue Alliance (Tag 20)");
        telemetry.addLine("  D-PAD RIGHT = Red Alliance (Tag 24)");
        telemetry.addLine("  D-PAD UP/DOWN = Manual Power Adjust");
        telemetry.addLine("========================================");
        telemetry.addLine("Press START to begin");
        telemetry.update();
    }

    // ========== LIMELIGHT DISTANCE & LAUNCH POWER CALCULATION ==========

    /**
     * Calculates distance from Limelight to AprilTag using trigonometry
     * Based on: distance = (targetHeight - limelightHeight) / tan(mountAngle + ty)
     */
    private double calculateDistanceFromLimelight(LLResultTypes.FiducialResult fiducial) {
        double ty = fiducial.getTargetYDegrees();
        double tyRad = Math.toRadians(ty);

        double heightDifference = TARGET_HEIGHT_MM - LIMELIGHT_HEIGHT_MM;
        double angleSum = LIMELIGHT_ANGLE_RAD + tyRad;

        if (Math.abs(Math.tan(angleSum)) < 0.001) {
            return 3000.0; // Default fallback
        }

        double horizontalDistance = heightDifference / Math.tan(angleSum);
        return Math.abs(horizontalDistance);
    }

    /**
     * Calculates required launch velocity using projectile motion
     * Formula: v₀ = √[g·d² / (2·cos²(θ)·(d·tan(θ) - Δy))]
     */
    private double calculateLaunchVelocity(double horizontalDistance) {
        double deltaY = TARGET_HEIGHT_MM - LAUNCH_HEIGHT_MM;
        double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);
        double tanTheta = Math.tan(LAUNCH_ANGLE_RAD);

        double numerator = GRAVITY_MM_S2 * horizontalDistance * horizontalDistance;
        double denominator = 2 * cosTheta * cosTheta * (horizontalDistance * tanTheta - deltaY);

        if (denominator <= 0) {
            return 3000.0; // Fallback velocity
        }

        return Math.sqrt(numerator / denominator);
    }

    /**
     * Converts linear velocity (mm/s) to motor RPM
     */
    private double velocityToRPM(double velocityMmPerS) {
        double wheelCircumferenceMm = Math.PI * WHEEL_DIAMETER_MM;
        double rps = velocityMmPerS / wheelCircumferenceMm;
        return rps * 60.0;
    }

    /**
     * Main function: Reads Limelight, calculates distance, and sets LAUNCH_MOTOR_POWER
     * Called continuously in the main loop
     */
    private void calculateAndSetLaunchPower() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid() && result.getFiducialResults().size() > 0) {
            int targetTagId = targetingRed ? TARGET_APRILTAG_ID_RED : TARGET_APRILTAG_ID_BLUE;

            for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetTagId) {
                    // Calculate distance
                    double distance = calculateDistanceFromLimelight(fiducial);

                    // Calculate required velocity and RPM
                    double requiredVelocity = calculateLaunchVelocity(distance);
                    double requiredRPM = velocityToRPM(requiredVelocity);

                    // Convert to motor power (0.0 to 1.0)
                    double calculatedPower = requiredRPM / MAX_MOTOR_RPM;
                    LAUNCH_MOTOR_POWER = Range.clip(calculatedPower, 0.0, 1.0);

                    return; // Target found, exit
                }
            }
        }

        // No target found - use default power
        LAUNCH_MOTOR_POWER = 0.8;
    }

    // ========== LAUNCH & INTAKE ==========

    private void launchBalls(int count) {
        for (int i = 0; i < count; i++) {
            intakeMotor.setPower(-1);
            sleep(600);
            intakeMotor.setPower(0);
            spatulaServo.setPosition(0.0);
            sleep(1000);
            spatulaServo.setPosition(1.0);
            sleep(200);
        }
    }

    // ========== ODOMETRY API ==========

    public int getXOdoPosition() {
        return xodo.getCurrentPosition();
    }

    public int getYOdoPosition() {
        return yodo.getCurrentPosition();
    }

    public double getXOdoInches() {
        return -xodo.getCurrentPosition() * ODOMETRY_INCHES_PER_TICK;
    }

    public double getYOdoInches() {
        return yodo.getCurrentPosition() * ODOMETRY_INCHES_PER_TICK;
    }

    public void resetOdometryPods() {
        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ========== NAVIGATION ==========

    private void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < JOYSTICK_DEADZONE ? 0 : value;
    }

    public boolean driveToPositionOdoWheels(double targetXInches, double targetYInches,
                                            double targetHeadingDeg, double maxSpeed,
                                            double timeoutSeconds) {
        final double FIELD_MIN_INCHES = -72.0;
        final double FIELD_MAX_INCHES = 72.0;
        final double POSITION_TOLERANCE = 1.0;
        final double HEADING_TOLERANCE = 2.0;
        final double KP_DRIVE = 0.03;
        final double KP_HEADING = 0.02;
        final double MIN_POWER = 0.15;
        final double SLOWDOWN_DIST = 12.0;

        targetXInches = Range.clip(targetXInches, FIELD_MIN_INCHES, FIELD_MAX_INCHES);
        targetYInches = Range.clip(targetYInches, FIELD_MIN_INCHES, FIELD_MAX_INCHES);

        double targetHeadingRad = Math.toRadians(targetHeadingDeg);
        resetOdometryPods();

        double startXOdo = getXOdoInches();
        double startYOdo = getYOdoInches();
        double startTime = getRuntime();
        boolean targetReached = false;

        while (opModeIsActive() && (getRuntime() - startTime) < timeoutSeconds && !targetReached) {
            double currentXInches = getXOdoInches() - startXOdo;
            double currentYInches = getYOdoInches() - startYOdo;
            double currentHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double currentHeadingDeg = Math.toDegrees(currentHeadingRad);

            double errorX = targetXInches - currentXInches;
            double errorY = targetYInches - currentYInches;
            double distance = Math.sqrt(errorX * errorX + errorY * errorY);

            double headingError = targetHeadingDeg - currentHeadingDeg;
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            if (distance < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE) {
                targetReached = true;
                break;
            }

            double angleToTarget = Math.atan2(errorX, errorY);
            double speedScale = maxSpeed;
            if (distance < SLOWDOWN_DIST) {
                speedScale = MIN_POWER + (maxSpeed - MIN_POWER) * (distance / SLOWDOWN_DIST);
            }

            double robotAngle = angleToTarget - currentHeadingRad;
            double driveX = Math.sin(robotAngle) * speedScale * KP_DRIVE * distance;
            double driveY = Math.cos(robotAngle) * speedScale * KP_DRIVE * distance;

            driveX = Range.clip(driveX, -maxSpeed, maxSpeed);
            driveY = Range.clip(driveY, -maxSpeed, maxSpeed);

            if (distance > POSITION_TOLERANCE) {
                if (Math.abs(driveX) < MIN_POWER && Math.abs(driveX) > 0.01) {
                    driveX = Math.signum(driveX) * MIN_POWER;
                }
                if (Math.abs(driveY) < MIN_POWER && Math.abs(driveY) > 0.01) {
                    driveY = Math.signum(driveY) * MIN_POWER;
                }
            }

            double rotationPower = headingError * KP_HEADING;
            rotationPower = Range.clip(rotationPower, -maxSpeed * 0.5, maxSpeed * 0.5);

            double frontLeftPower = -driveY + driveX + rotationPower;
            double backLeftPower = -driveY - driveX + rotationPower;
            double frontRightPower = -driveY - driveX - rotationPower;
            double backRightPower = -driveY + driveX - rotationPower;

            double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > maxSpeed) {
                double scale = maxSpeed / maxPower;
                frontLeftPower *= scale;
                backLeftPower *= scale;
                frontRightPower *= scale;
                backRightPower *= scale;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addLine("--- Odo Wheel Navigation ---");
            telemetry.addData("Target", "X: %.1f, Y: %.1f, H: %.1f°",
                    targetXInches, targetYInches, targetHeadingDeg);
            telemetry.addData("Current", "X: %.1f, Y: %.1f, H: %.1f°",
                    currentXInches, currentYInches, currentHeadingDeg);
            telemetry.addData("Distance", "%.2f in", distance);
            telemetry.update();
        }

        stopDriveMotors();
        return targetReached;
    }

    public boolean driveToPositionOdoWheels(double targetXInches, double targetYInches) {
        return driveToPositionOdoWheels(targetXInches, targetYInches, 0.0, 0.5, 5.0);
    }

    public boolean driveToPositionOdoWheels(double targetXInches, double targetYInches,
                                            double targetHeadingDeg) {
        return driveToPositionOdoWheels(targetXInches, targetYInches, targetHeadingDeg, 0.5, 5.0);
    }
}

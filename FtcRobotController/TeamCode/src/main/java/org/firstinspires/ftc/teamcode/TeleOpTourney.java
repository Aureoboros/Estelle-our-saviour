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

@TeleOp(name = "TeleOpTourney")
public class TeleOpTourney extends LinearOpMode {

    // Motor power constants
    private static final double INTAKE_POWER = 1.0;
    private static final double LAUNCH_MOTOR_POWER = 0.7;
    private static final double SPATULA_SERVO_POWER = 0.8;

    // Navigation constants
    private static final double AUTO_MAX_SPEED = 0.7;
    private static final double AUTO_MIN_SPEED = 0.15;
    private static final double SLOWDOWN_DISTANCE_FEET = 2.0;
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.5;

    // Limelight/Launch constants
    private static final double LAUNCH_HEIGHT_MM = 304.143;
    private static final double TARGET_HEIGHT_ABOVE_TAG_MM = 152.4; // 6 inches
    private static final double LAUNCH_ANGLE_DEG = 54.0;
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);
    private static final double GRAVITY_MM_S2 = 9810.0;
    private static final double MAX_MOTOR_RPM = 6000.0;
    private static final double FIELD_SIZE_MM = 3657.6; // 12 feet in mm
    private static final double WHEEL_DIAMETER_MM = 100.0; // CALIBRATE THIS
    private static final int TARGET_APRILTAG_ID = 24;  // Red alliance goal

    // DECODE Season AprilTag positions (in mm from field center)
    private static final double[][] TAG_POSITIONS = {
            {0.0, FIELD_SIZE_MM / 2},      // Tag 20 - Blue Alliance Goal
            {0.0, 1828.8},                 // Tag 21 - Neutral (0, 6ft)
            {0.0, 1828.8},                 // Tag 22 - Neutral (0, 6ft)
            {0.0, 1828.8},                 // Tag 23 - Neutral (0, 6ft)
            {0.0, -FIELD_SIZE_MM / 2}      // Tag 24 - Red Alliance Goal
    };

    // Field positions (in feet, measured from field center) - MIRRORED FOR RED
    private static final double RED_LOWER_SPIKE_X = 3.0;   // Mirrored X
    private static final double RED_LOWER_SPIKE_Y = -3.0;   // Mirrored Y

    private static final double RED_MIDDLE_SPIKE_X = 3.0;  // Mirrored X
    private static final double RED_MIDDLE_SPIKE_Y = -1.0;  // Mirrored Y

    private static final double RED_TOP_SPIKE_X = 3.0;     // Mirrored X
    private static final double RED_TOP_SPIKE_Y = 1.0;    // Mirrored Y

    private static final double RED_SHOOT_X = 0.0;         // Same X
    private static final double RED_SHOOT_Y = 4.0;        // Mirrored Y

    private static final double OBELISK_X = 0.0;
    private static final double OBELISK_Y = 6.0;

    private static final double RED_DEFAULT_START_X = 1.0;  // Mirrored X
    private static final double RED_DEFAULT_START_Y = -5.0;  // Mirrored Y

    private static final double FIELD_MIN_FEET = -6.0;
    private static final double FIELD_MAX_FEET = 6.0;

    // Odometry constants
    private static final double ODOMETRY_INCHES_PER_TICK = 0.001; // CALIBRATE THIS
    private static final double COUNTS_PER_MM = 1.0; // CALIBRATE THIS

    // Tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private boolean odometryInitialized = false;
    private int lastLeftEncoderPos = 0;
    private int lastRightEncoderPos = 0;
    private int lastStrafeEncoderPos = 0;
    private boolean positionDetected = false;

    // ========== MOTOR POWER CONSTANTS ==========
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;

    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 1.0;  // Full power for driver 1
    private static final double GAMEPAD2_MAX_POWER = 0.5;  // Half power for driver 2


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
    private static final double COUNTS_PER_MOTOR_REV = 537.7;  // REV HD Hex Motor
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
    private TeleOpTourney.SpeedMode currentSpeedMode = TeleOpTourney.SpeedMode.FAST;

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
            updateDriveEncoderOdometry(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);

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

            // ========== START BUTTON - SLOW MODE TOGGLE ==========
            if (startPressed) {
                slowMode = !slowMode;
            }

            // ========== Y BUTTON - FIELD CENTRIC TOGGLE ==========
            if (yPressed) {
                fieldCentric = !fieldCentric;
            }

            // ========== A/B/X BUTTONS - SPEED PRESETS ==========
            if (aPressed) {
                currentSpeedMode = TeleOpTourney.SpeedMode.SLOW;
            } else if (bPressed) {
                currentSpeedMode = TeleOpTourney.SpeedMode.MEDIUM;
            } else if (xPressed) {
                currentSpeedMode = TeleOpTourney.SpeedMode.FAST;
            }

            // ========== BACK BUTTON - RESET ODOMETRY ==========
            if (backPressed) {
                imu.resetYaw();
                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robotX = 0.0;
                robotY = 0.0;
                robotHeading = 0.0;
                lastFLEncoder = 0;
                lastBLEncoder = 0;
                lastFREncoder = 0;
                lastBREncoder = 0;
            }

            // ========== LEFT BUMPER - RESET IMU HEADING ==========
            if (leftBumperPressed) {
                imu.resetYaw();
                robotHeading = 0.0;

                //TEST CODES Just for testing in one class
                currentPos = spatulaServo.getPosition();
                spatulaServo.setPosition(0.5);
                sleep(1000);
                spatulaServo.setPosition(0);
                sleep(1000);
                spatulaServo.setPosition(1);
                //    currentPos = spinSpinServo.getPosition();
                //    spinSpinServo.setPosition(0.5);
                //    sleep(1000);
                //    spinSpinServo.setPosition(0);
                //    sleep(1000);
                //    spinSpinServo.setPosition(1);
                //    launchMotor.setPower(1);
                //    intakeMotor.setPower(1);

            }

            // ========== RIGHT BUMPER - SNAP TO 0° ==========
            if (rightBumperPressed) {
                // Calculate rotation needed to face forward
                double targetHeading = 0.0;
                double currentHeading = robotHeading;
                double headingError = targetHeading - currentHeading;

                // Normalize to [-PI, PI]
                while (headingError > Math.PI) headingError -= 2 * Math.PI;
                while (headingError < -Math.PI) headingError += 2 * Math.PI;

                // Quick snap rotation (will execute over next few loops)
                // This just sets up for the next control cycle
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
                    currentGamepad1.dpad_up || currentGamepad1.dpad_down ||
                    currentGamepad1.dpad_left || currentGamepad1.dpad_right;

            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE ||
                    currentGamepad2.dpad_up || currentGamepad2.dpad_down ||
                    currentGamepad2.dpad_left || currentGamepad2.dpad_right;

            if (gamepad1Active) {
                // Joystick control
                y = currentGamepad1.left_stick_y;
                x = currentGamepad1.right_stick_x;
                rx = currentGamepad1.right_trigger - currentGamepad1.left_trigger;

                // DPAD precision control (overrides joystick)
                if (currentGamepad1.dpad_up) {
                    y = -0.3;  // Forward
                    x = 0;
                    rx = 0;
                } else if (currentGamepad1.dpad_down) {
                    y = 0.3;  // Backward
                    x = 0;
                    rx = 0;
                } else if (currentGamepad1.dpad_left) {
                    y = 0;
                    x = -0.3;  // Strafe left
                    rx = 0;
                } else if (currentGamepad1.dpad_right) {
                    y = 0;
                    x = 0.3;  // Strafe right
                    rx = 0;
                }

                maxDrivePower = GAMEPAD1_MAX_POWER;
                activeDriver = "DRIVER 1";
            } else if (gamepad2Active) {
                // Joystick control
                y = currentGamepad2.left_stick_y;
                x = currentGamepad2.right_stick_x;
                rx = currentGamepad2.right_trigger - currentGamepad2.left_trigger;

                // DPAD precision control (overrides joystick)
                if (currentGamepad2.dpad_up) {
                    y = -0.3;  // Forward
                    x = 0;
                    rx = 0;
                } else if (currentGamepad2.dpad_down) {
                    y = 0.3;  // Backward
                    x = 0;
                    rx = 0;
                } else if (currentGamepad2.dpad_left) {
                    y = 0;
                    x = -0.3;  // Strafe left
                    rx = 0;
                } else if (currentGamepad2.dpad_right) {
                    y = 0;
                    x = 0.3;  // Strafe right
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

            // Apply slow mode (overrides speed preset)
            if (slowMode) {
                speedMultiplier = SLOW_MODE_MULTIPLIER;
            }

            x *= speedMultiplier;
            y *= speedMultiplier;
            rx *= speedMultiplier;

            // ========== FIELD CENTRIC CONVERSION ==========
            if (fieldCentric) {
                // Convert robot-centric to field-centric
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

            // Normalize and scale
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

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ========== TELEMETRY ==========
            telemetry.addLine("========== DRIVE ONLY MODE ==========");
            telemetry.addData("Active Driver", activeDriver);

            // Speed mode display
            String speedModeStr = slowMode ? "SLOW MODE (30%)" :
                    currentSpeedMode == TeleOpTourney.SpeedMode.SLOW ? "SLOW (30%)" :
                            currentSpeedMode == TeleOpTourney.SpeedMode.MEDIUM ? "MEDIUM (60%)" :
                                    "FAST (100%)";
            telemetry.addData("Speed", speedModeStr);
            telemetry.addData("Control Mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addLine();

            telemetry.addLine("--- Position (Odometry) ---");
            telemetry.addData("X Position", "%.2f inches", robotX);
            telemetry.addData("Y Position", "%.2f inches", robotY);
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(robotHeading));
            telemetry.addLine();

            telemetry.addLine("--- Drive Powers ---");
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BR", "%.2f", backRightPower);
            telemetry.addLine();

            telemetry.addLine("--- Controls ---");
            telemetry.addLine("A/B/X = Speed Presets (Slow/Med/Fast)");
            telemetry.addLine("START = Toggle Slow Mode");
            telemetry.addLine("Y = Toggle Field-Centric");
            telemetry.addLine("LB = Reset Heading | RB = Snap to 0°");
            telemetry.addLine("BACK = Reset Position");
            telemetry.addLine("DPAD = Precision Movement");

            telemetry.addData("Spin = ", "%.2f", currentPos);

            telemetry.update();

        }

        }

    private void initializeHardware() {
        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Initialize other motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        // Initialize odometry motors
        xodo = hardwareMap.get(DcMotor.class, "xOdo");
        yodo = hardwareMap.get(DcMotor.class, "yOdo");

        // Initialize servos
        spinSpinServo = hardwareMap.get(Servo.class, "spinSpinServo");
        spatulaServo = hardwareMap.get(Servo.class, "spatulaServo");
        stopServo = hardwareMap.get(Servo.class, "stopServo");

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servo positions
        stopServo.setPosition(1.0); // Closed
        spatulaServo.setPosition(0.0); // Down
        spinSpinServo.setPosition(0.0);


        // ========== INIT TELEMETRY ==========
        telemetry.addLine("========================================");
        telemetry.addLine("TELEOP");
        telemetry.addLine("========================================");
        telemetry.addData("Drive Motors", "✓ (4 motors)");
        telemetry.addData("Odometry", "✓ (Drive Encoders)");
        telemetry.addData("IMU", "✓");
        telemetry.addLine("========================================");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick Y = Forward/Back");
        telemetry.addLine("  Right Stick X = Strafe L/R");
        telemetry.addLine("  LT/RT = Rotate L/R");
        telemetry.addLine("  DPAD UP/DOWN/LEFT/RIGHT = Precise Movement");
        telemetry.addLine("  A/B/X = Speed Presets");
        telemetry.addLine("  START = Toggle Slow Mode");
        telemetry.addLine("  BACK = Reset Position");
        telemetry.addLine("========================================");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("========================================");
        telemetry.update();
    }

    private void detectInitialPosition() {
        telemetry.addLine("Detecting initial position...");
        telemetry.update();

        for (int i = 0; i < 30 && !positionDetected; i++) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && result.getFiducialResults().size() > 0) {
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    int tagId = fiducial.getFiducialId();

                    // Use any visible tag to establish position
                    if (tagId >= 20 && tagId <= 24) {
                        double distance = calculateDistanceFromLimelight(fiducial);
                        double angle = Math.toRadians(fiducial.getTargetXDegrees());

                        // Calculate robot position relative to tag
                        double[] tagPos = TAG_POSITIONS[tagId - 20];
                        robotX = (tagPos[0] - distance * Math.sin(angle)) / 304.8; // mm to feet
                        robotY = (tagPos[1] - distance * Math.cos(angle)) / 304.8;

                        positionDetected = true;
                        telemetry.addData("Position detected via Tag", tagId);
                        telemetry.addData("Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
                        telemetry.update();
                        break;
                    }
                }
            }
            sleep(100);
        }
    }

    private void autoAimAndShoot() {
        telemetry.addLine("Auto-aiming at RED target (Tag 24)...");
        telemetry.update();

        boolean targetAcquired = false;
        double timeout = 3.0; // 3 second timeout
        double startTime = getRuntime();

        while (opModeIsActive() && !targetAcquired && (getRuntime() - startTime) < timeout) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && result.getFiducialResults().size() > 0) {
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    if (fiducial.getFiducialId() == TARGET_APRILTAG_ID) {
                        // Calculate distance using both methods
                        double limelightDistance = calculateDistanceFromLimelight(fiducial);
                        double odoDistance = confirmPositionWithOdometry(TARGET_APRILTAG_ID);
                        double finalDistance = (limelightDistance + odoDistance) / 2.0;

                        // Calculate required launch velocity and RPM
                        double requiredVelocity = calculateLaunchVelocity(finalDistance);
                        double requiredRPM = velocityToRPM(requiredVelocity);

                        // Aim turret
                        aimTurretAtTag(fiducial);

                        // Set launcher speed
                        setLauncherSpeed(requiredRPM);
                        sleep(500); // Allow launcher to spin up

                        // Launch sequence
                        launchBalls(3); // Launch 3 balls

                        targetAcquired = true;

                        telemetry.addData("Target Acquired", "Tag %d (RED)", TARGET_APRILTAG_ID);
                        telemetry.addData("Distance (mm)", "%.0f", finalDistance);
                        telemetry.addData("Required RPM", "%.0f", requiredRPM);
                        telemetry.update();
                        break;
                    }
                }
            }
            sleep(50);
        }

        if (!targetAcquired) {
            telemetry.addLine("⚠ Target not found - shooting with default settings");
            telemetry.update();
            setLauncherSpeed(MAX_MOTOR_RPM * 0.7);
            sleep(500);
            launchBalls(3);
        }

        // Stop launcher
        launchMotor.setPower(0);
    }

    private double calculateDistanceFromLimelight(LLResultTypes.FiducialResult fiducial) {
        // Get target area for distance estimation
        double area = fiducial.getTargetArea();

        // Rough distance estimation (CALIBRATE THIS based on your setup)
        // This is a placeholder formula - you'll need to calibrate
        double distance = 5000.0 / Math.sqrt(area);

        return distance;
    }

    private double confirmPositionWithOdometry(int tagId) {
        if (tagId < 20 || tagId > 24) return 0;

        double[] tagPos = TAG_POSITIONS[tagId - 20];
        double robotXmm = robotX * 304.8; // feet to mm
        double robotYmm = robotY * 304.8;

        double dx = tagPos[0] - robotXmm;
        double dy = tagPos[1] - robotYmm;

        return Math.sqrt(dx * dx + dy * dy);
    }

    private double calculateLaunchVelocity(double horizontalDistance) {
        // Projectile motion: v₀ = √[g·d² / (2·cos²(θ)·(d·tan(θ) - Δy))]
        double deltaY = TARGET_HEIGHT_ABOVE_TAG_MM - LAUNCH_HEIGHT_MM;
        double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);
        double tanTheta = Math.tan(LAUNCH_ANGLE_RAD);

        double numerator = GRAVITY_MM_S2 * horizontalDistance * horizontalDistance;
        double denominator = 2 * cosTheta * cosTheta * (horizontalDistance * tanTheta - deltaY);

        if (denominator <= 0) {
            // Safety fallback
            return 3000.0; // Default velocity in mm/s
        }

        return Math.sqrt(numerator / denominator);
    }

    private double velocityToRPM(double velocityMmPerS) {
        double wheelCircumferenceMm = Math.PI * WHEEL_DIAMETER_MM;
        double rps = velocityMmPerS / wheelCircumferenceMm;
        return rps * 60.0;
    }

    private void aimTurretAtTag(LLResultTypes.FiducialResult fiducial) {
        double tx = fiducial.getTargetXDegrees();

        // Convert angle to servo position (5 turn servo = 1800° range)
        double currentPos = spinSpinServo.getPosition();
        double angleOffset = tx / 1800.0;

        double newPos = Range.clip(currentPos + angleOffset, 0.0, 1.0);
        spinSpinServo.setPosition(newPos);

        sleep(300); // Allow turret to settle
    }

    private void setLauncherSpeed(double rpm) {
        double power = Range.clip(rpm / MAX_MOTOR_RPM, 0.0, 1.0);
        launchMotor.setPower(power);
    }

    private void launchBalls(int count) {
        for (int i = 0; i < count; i++) {
            // Open stopper to allow ball through
            stopServo.setPosition(0.0);
            sleep(100);

            // Actuate spatula to push ball
            spatulaServo.setPosition(1.0);
            sleep(300);
            spatulaServo.setPosition(0.0);
            sleep(200);

            // Close stopper
            stopServo.setPosition(1.0);
            sleep(200);
        }
    }

    private void updateDriveEncoderOdometry(DcMotor fl, DcMotor bl, DcMotor fr, DcMotor br, IMU imu) {
        int currentFL = fl.getCurrentPosition();
        int currentBL = bl.getCurrentPosition();
        int currentFR = fr.getCurrentPosition();
        int currentBR = br.getCurrentPosition();

        int deltaFL = currentFL - lastFLEncoder;
        int deltaBL = currentBL - lastBLEncoder;
        int deltaFR = currentFR - lastFREncoder;
        int deltaBR = currentBR - lastBREncoder;

        double avgForward = (deltaFL + deltaBL + deltaFR + deltaBR) / 4.0;
        double avgStrafe = (-deltaFL + deltaBL + deltaFR - deltaBR) / 4.0;

        double deltaForwardInches = avgForward / COUNTS_PER_INCH;
        double deltaStrafeInches = avgStrafe / COUNTS_PER_INCH;

        robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        robotX += deltaForwardInches * Math.sin(robotHeading) + deltaStrafeInches * Math.cos(robotHeading);
        robotY += deltaForwardInches * Math.cos(robotHeading) - deltaStrafeInches * Math.sin(robotHeading);

        lastFLEncoder = currentFL;
        lastBLEncoder = currentBL;
        lastFREncoder = currentFR;
        lastBREncoder = currentBR;
    }

    private double applyDeadzone(double value) {
        return Math.abs(value) < JOYSTICK_DEADZONE ? 0 : value;
    }
}


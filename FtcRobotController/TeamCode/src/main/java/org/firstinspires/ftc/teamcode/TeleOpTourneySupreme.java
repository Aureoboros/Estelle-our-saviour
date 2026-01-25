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
// Buttons as per https://docs.google.com/document/d/1_6g9IvvFj1Ofdy_aqY4_loDmhZa_UZqgFy8Ihcow7NI/edit?tab=t.0
@TeleOp(name = "TeleOpTourneySupreme")
public class TeleOpTourneySupreme extends LinearOpMode {

    // Motor power constants
    private static final double INTAKE_POWER = -1.0;
    private static final double LAUNCH_MOTOR_POWER_HIGH = 0.9;
    private static final double LAUNCH_MOTOR_POWER_LOW = 0.4;
    private static final double SPATULA_SERVO_POWER = 0.8;

    // Navigation constants
    private static final double AUTO_MAX_SPEED = 0.7;
    private static final double AUTO_MIN_SPEED = 0.15;
    private static final double SLOWDOWN_DISTANCE_FEET = 2.0;
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.5;

    // Limelight/Launch constants
    private static final double LAUNCH_HEIGHT_MM = 304.143;      // Height of launcher from ground (12 inches)
    private static final double TAG_HEIGHT_MM = 750.0;           // Height of AprilTag center from ground (calibrated)
    private static final double TARGET_HEIGHT_MM = 900.0;        // Target height (goal opening) - above the tag
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

    // Odometry constants (goBILDA Odometry Pod with 35mm wheel)
    // Calculation: (π × 1.378 inches) / 2000 CPR = 0.002164 inches per tick
    private static final double ODOMETRY_INCHES_PER_TICK = 0.002164;
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

    private double launchMotorPower = LAUNCH_MOTOR_POWER_HIGH;

    private int togglestopper = 1; // B toggle to open/close stopper, default close

    private int toggleintake = 1; // Y toggle intake default started

    private int togglespatula = 0; // toggle spatula, default down

    private int togglemotorpower = 1; // Back toggles launch motor power between high & low, default high for shooting from far

    private int tagid = 24; //red by default

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
//        detectInitialPosition();
        boolean found = false;
        int count = 0;
        double spinpos = 0.05;
        do {
            spinSpinServo.setPosition(spinpos);
            sleep(100);
            found = detectInitialPosition();
            spinpos = spinpos + 0.05;
        } while ((!found)&&(spinpos <=0.3));


        // Initialize gamepad state tracking
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Toggle states
        boolean slowMode = false;
        boolean fieldCentric = false;

        waitForStart();
        if (isStopRequested()) return;
        launchMotor.setPower(LAUNCH_MOTOR_POWER_HIGH);
        intakeMotor.setPower(INTAKE_POWER);
        stopServo.setPosition(0.5);

        while (opModeIsActive()) {
            // ========== UPDATE GAMEPAD STATES ==========
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // ========== UPDATE ODOMETRY ==========
            updateDriveEncoderOdometry(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);
            //robotX = getXOdoInches() / 12;
            //robotY = getYOdoInches() / 12;
            // ARATRIKA COME BACK TO THIS >:( also written by aratrika yeah

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
                if (togglespatula == 1) {
                    spatulaServo.setPosition(0);
                    togglespatula = 0;
                }
                else {
                    spatulaServo.setPosition(1);
                    togglespatula = 1;
                }
            }

            // ========== Y BUTTON - FIELD CENTRIC TOGGLE ==========
            if (yPressed) {
                if (toggleintake == 1) {
                    intakeMotor.setPower(INTAKE_POWER);
                    toggleintake = 0;
                }
                else {
                    intakeMotor.setPower(0);
                    toggleintake = 1;
                }
            }

            // ========== A/B/X BUTTONS - SPEED PRESETS ==========
            if (aPressed) {
                //driveToPosition(0,0,0);
                // driveToPositionOdoWheels(0, 0);
                //shoot position 0, -4
                driveToPosition(0, -4, 0);
            }
            if (bPressed) {
                //Park position
                if(tagid == 20) {
                    driveToPosition(3, -3, 0);
                }
                else {
                    driveToPosition(-3,-3, 0);
                }
//                if (togglestopper == 1) {
//                    stopServo.setPosition(0.5);
//                    togglestopper = 0;
//                }
//                else {
//                    stopServo.setPosition(1.0);
//                    togglestopper = 1;
//                }
            }
            if (xPressed) {
            //    launchBalls(1);
                autoAimAndShoot();
            }

            // ========== BACK BUTTON - RESET ODOMETRY ==========
            if (backPressed) {
//                imu.resetYaw();
//                frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                robotX = 0.0;
//                robotY = 0.0;
//                robotHeading = 0.0;
//                lastFLEncoder = 0;
//                lastBLEncoder = 0;
//                lastFREncoder = 0;
//                lastBREncoder = 0;
//                COME BACK TO THIS TOO ARATRIKA...PLEASE...



                //resetOdometryPods();
                if(togglemotorpower == 1) {
                    launchMotorPower = LAUNCH_MOTOR_POWER_HIGH;
                    togglemotorpower = 0;
                    launchMotor.setPower(launchMotorPower);
                }
                else {
                    launchMotorPower = LAUNCH_MOTOR_POWER_LOW;
                    togglemotorpower = 1;
                    launchMotor.setPower(launchMotorPower);
                }
            }

            // ========== LEFT BUMPER - RESET IMU HEADING ==========
            if (leftBumperPressed) {
                imu.resetYaw();
                robotHeading = 0.0;
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
//                if (currentGamepad1.dpad_up) {
//                    y = -0.3;  // Forward
//                    x = 0;
//                    rx = 0;
//                } else if (currentGamepad1.dpad_down) {
//                    y = 0.3;  // Backward
//                    x = 0;
//                    rx = 0;
//                } else if (currentGamepad1.dpad_left) {
//                    y = 0;
//                    x = -0.3;  // Strafe left
//                    rx = 0;
//                } else if (currentGamepad1.dpad_right) {
//                    y = 0;
//                    x = 0.3;  // Strafe right
//                    rx = 0;
//                }
                if (currentGamepad1.dpad_left) {
                    tagid = 20;
                }
                else if (currentGamepad1.dpad_right) {
                    tagid = 24;
                }
                if (currentGamepad1.dpad_up) {
                    launchMotorPower += 0.05;
                }
                else if (currentGamepad1.dpad_down) {
                    launchMotorPower -= 0.05;
                }

                maxDrivePower = GAMEPAD1_MAX_POWER;
                activeDriver = "DRIVER 1";
            } else if (gamepad2Active) {
                // Joystick control
                y = currentGamepad2.left_stick_y;
                x = currentGamepad2.right_stick_x;
                rx = currentGamepad2.right_trigger - currentGamepad2.left_trigger;

                // DPAD precision control (overrides joystick)
//                if (currentGamepad2.dpad_up) {
//                    y = -0.3;  // Forward
//                    x = 0;
//                    rx = 0;
//                } else if (currentGamepad2.dpad_down) {
//                    y = 0.3;  // Backward
//                    x = 0;
//                    rx = 0;
//                } else if (currentGamepad2.dpad_left) {
//                    y = 0;
//                    x = -0.3;  // Strafe left
//                    rx = 0;
//                } else if (currentGamepad2.dpad_right) {
//                    y = 0;
//                    x = 0.3;  // Strafe right
//                    rx = 0;
//                }
                if (currentGamepad1.dpad_left) {
                    tagid = 20;
                }
                else if (currentGamepad1.dpad_right) {
                    tagid = 24;
                }
                if (currentGamepad1.dpad_up) {
                    launchMotorPower += 0.05;
                }
                else if (currentGamepad1.dpad_down) {
                    launchMotorPower -= 0.05;
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
            frontLeftMotor.setPower(frontLeftPower/2);
            backLeftMotor.setPower(backLeftPower/2);
            frontRightMotor.setPower(frontRightPower/2);
            backRightMotor.setPower(backRightPower/2);

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
            telemetry.addData("LaunchMotorPower ", "%f", launchMotorPower);
            telemetry.addLine();

            telemetry.addLine("--- Controls ---");
            telemetry.addLine("A/B/X = Speed Presets (Slow/Med/Fast)");
            telemetry.addLine("START = Toggle Slow Mode");
            telemetry.addLine("Y = Toggle Field-Centric");
            telemetry.addLine("LB = Reset Heading | RB = Snap to 0°");
            telemetry.addLine("BACK = Reset Position");
            telemetry.addLine("DPAD = Precision Movement");

            telemetry.addData("Spin = ", "%.2f", currentPos);
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

        // Initialize Limelight in 3D mode for AprilTag pose estimation
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // Pipeline 0 should be configured for AprilTag 3D in Limelight web interface
        limelight.setPollRateHz(100); // Set polling rate for responsive tracking
        limelight.start();
        
        // Initialize robot orientation for 3D localization
        // This should be called periodically with IMU heading for MegaTag2
        limelight.updateRobotOrientation(0.0);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servo positions
        stopServo.setPosition(0.5); // Closed
        spatulaServo.setPosition(1.0); // Down
        spinSpinServo.setPosition(0.01); // Stop the spinservo to turn too far
        launchMotor.setPower(LAUNCH_MOTOR_POWER_HIGH);



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

    private boolean detectInitialPosition() {
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
        return positionDetected;
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
                    if (fiducial.getFiducialId() == tagid) {
                        // Calculate distance using both methods
                        double limelightDistance = calculateDistanceFromLimelight(fiducial);
                        double odoDistance = confirmPositionWithOdometry(tagid);
                        double finalDistance = (limelightDistance + odoDistance) / 2.0;

                        // Calculate required launch velocity and RPM
                        double requiredVelocity = calculateLaunchVelocity(finalDistance);
                        double requiredRPM = velocityToRPM(requiredVelocity);

                        // Aim turret
                        aimTurretAtTag(fiducial);

                        // Set launcher speed
                        //setLauncherSpeed(requiredRPM);
                        launchMotor.setPower(launchMotorPower);
                        sleep(1000); // Allow launcher to spin up

                        // Launch sequence
                        launchBalls(1); // Launch 3 balls

                        targetAcquired = true;

                        telemetry.addData("Target Acquired", "Tag %d ", tagid);
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
            //setLauncherSpeed(MAX_MOTOR_RPM * 0.9);
            launchMotor.setPower(launchMotorPower);
            sleep(500);
            launchBalls(1);
        }

        // Stop launcher
        //launchMotor.setPower(0);
    }

    private double calculateDistanceFromLimelight(LLResultTypes.FiducialResult fiducial) {
        // Method 1: Use target area (requires calibration)
        double area = fiducial.getTargetArea();
        
        // Method 2: Use TY (vertical angle) for more accurate distance
        // This uses the known height difference between camera and tag
        double ty = fiducial.getTargetYDegrees();

        // Camera mounting height and angle (CALIBRATE THESE FOR YOUR ROBOT)
        final double CAMERA_HEIGHT_MM = 330.0;  // Height of camera lens from ground
        final double CAMERA_ANGLE_DEG = 5.0;    // Camera tilt angle (positive = tilted up)
        final double TAG_HEIGHT_MM = 750.0;     // Height of AprilTag center from ground
        
        // Calculate distance using trigonometry
        // distance = (tagHeight - cameraHeight) / tan(cameraAngle + ty)
        double angleToTargetRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);
        
        double distanceFromTY = 0.0;
        if (Math.abs(angleToTargetRad) > 0.01) {  // Avoid division by zero
            distanceFromTY = Math.abs((TAG_HEIGHT_MM - CAMERA_HEIGHT_MM) / Math.tan(angleToTargetRad));
        }
        
        // Method 3: Use area-based estimation as fallback
        // Formula: distance = k / sqrt(area), where k is calibration constant
        // CALIBRATE: Measure area at known distance, then k = distance * sqrt(area)
        final double AREA_CALIBRATION_CONSTANT = 5000.0;  // CALIBRATE THIS
        double distanceFromArea = AREA_CALIBRATION_CONSTANT / Math.sqrt(Math.max(area, 0.0001));
        
        // Use TY-based distance if valid, otherwise fall back to area
        double distance;
        if (distanceFromTY > 100.0 && distanceFromTY < 5000.0) {
            // TY-based distance seems reasonable
            distance = distanceFromTY;
            telemetry.addData("Distance Method", "TY-based: %.0f mm", distanceFromTY);
        } else {
            // Fall back to area-based
            distance = distanceFromArea;
            telemetry.addData("Distance Method", "Area-based: %.0f mm", distanceFromArea);
        }
        
        telemetry.addData("TY Angle", "%.2f°", ty);
        telemetry.addData("Target Area", "%.4f", area);
        
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
        // deltaY = target height - launch height (positive means target is above launcher)
        double deltaY = TARGET_HEIGHT_MM - LAUNCH_HEIGHT_MM;
        double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);
        double tanTheta = Math.tan(LAUNCH_ANGLE_RAD);

        double numerator = GRAVITY_MM_S2 * horizontalDistance * horizontalDistance;
        double denominator = 2 * cosTheta * cosTheta * (horizontalDistance * tanTheta - deltaY);

        // Debug telemetry for launch calculation
        telemetry.addData("Launch deltaY", "%.1f mm (target - launcher)", deltaY);
        telemetry.addData("Launch angle", "%.1f°", LAUNCH_ANGLE_DEG);
        telemetry.addData("Horizontal dist", "%.1f mm", horizontalDistance);

        if (denominator <= 0) {
            // Safety fallback - target unreachable with current angle
            telemetry.addData("Launch calc", "FALLBACK (denominator <= 0)");
            return 3000.0; // Default velocity in mm/s
        }

        double velocity = Math.sqrt(numerator / denominator);
        telemetry.addData("Calculated velocity", "%.1f mm/s", velocity);
        return velocity;
    }

    private double velocityToRPM(double velocityMmPerS) {
        double wheelCircumferenceMm = Math.PI * WHEEL_DIAMETER_MM;
        double rps = velocityMmPerS / wheelCircumferenceMm;
        return rps * 60.0;
    }

    private void aimTurretAtTag(LLResultTypes.FiducialResult fiducial) {
        double tx = fiducial.getTargetXDegrees();

        // Turret tracking constants for 5-turn servo with 0.0-0.5 range
        // The 0.0-0.5 range represents 180° of rotation (50% of 360°)
        // This allows full hemisphere coverage for target tracking
        final double TX_TOLERANCE = 2.0;        // Don't adjust if within 2 degrees
        final double SERVO_MIN = 0.0;           // Minimum servo position
        final double SERVO_MAX = 0.5;           // Maximum servo position for 180° rotation
        final double SERVO_CENTER = 0.25;       // Center position for turret (middle of 0.0-0.5)
        
        // Reduced gain for gentler, more controlled turret movement
        // 0.5 range / 180° = ~0.00278 per degree, using lower value for smooth tracking
        final double BASE_GAIN = 0.003;         // Reduced from 0.0125 for gentler movement
        final double GAIN_MULTIPLIER = 1.0;     // No multiplier
        final double PROPORTIONAL_GAIN = BASE_GAIN * GAIN_MULTIPLIER; // = 0.003
        final double MAX_STEP = 0.01;           // Reduced from 0.025 - max movement per adjustment

        // Skip if already on target
        if (Math.abs(tx) <= TX_TOLERANCE) {
            telemetry.addData("Turret", "ON TARGET (TX: %.1f°)", tx);
            return;
        }

        // Calculate adjustment (negative because we move toward target, opposite of offset)
        // TX positive = target is to the right = adjust servo position accordingly
        // Adjust sign based on your servo mounting direction
        double adjustment = Range.clip(-tx * PROPORTIONAL_GAIN, -MAX_STEP, MAX_STEP);
        
        double currentPos = spinSpinServo.getPosition();
        double newPos = currentPos + adjustment;

        // Clip to valid servo range (0.0 to 0.5)
        newPos = Range.clip(newPos, SERVO_MIN, SERVO_MAX);
        spinSpinServo.setPosition(newPos);

        telemetry.addData("Turret TX", "%.1f°", tx);
        telemetry.addData("Turret Adjustment", "%.4f (x%.1f)", adjustment, GAIN_MULTIPLIER);
        telemetry.addData("Turret Position", "%.3f → %.3f (range: %.1f-%.1f)", 
                currentPos, newPos, SERVO_MIN, SERVO_MAX);

        sleep(300); // Allow turret to settle
    }

    private void setLauncherSpeed(double rpm) {
        double power = Range.clip(rpm / MAX_MOTOR_RPM, 0.0, 1.0);
        //launchMotor.setPower(-1.0 * power);
        launchMotor.setPower(power);
        sleep(1000);
    }
    /*
     Current logic of launchBalls
     Launcher Motor always on
     Intake always on
     Spatula initial position down
     Open stopper for 600ms (loop begin)
     Close stopper
     Wait for stopper to reach closed state
     Set Spatula up for 600ms
     Spatula down
     Wait for Spatula to come down
     Go back to open stopper state (loop begin)
    */
    private void launchBalls(int count) {
        //launchMotor.setPower(LAUNCH_MOTOR_POWER_HIGH);
        //sleep(2000);
        for (int i = 0; i < count; i++) {
            // Open stopper to allow ball through
            stopServo.setPosition(1.0);
            sleep(1000);
            // Close stopper to stop other balls from going under the spatula
            stopServo.setPosition(0.5);
            //while (stopServo.getPosition() != 0.0);
            sleep(1000);
            // Actuate spatula to push ball
            spatulaServo.setPosition(0.0);
            sleep(1000);
            spatulaServo.setPosition(1.0);
            sleep(1000);
            //while (spatulaServo.getPosition() != 1.0);
            // Close stopper
            //stopServo.setPosition(1.0);
            //sleep(200);
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


    /**
     * Drives the robot to a specific position on the field using wheel encoders.
     * Uses a TWO-PHASE approach for large heading changes (>30°) to prevent oscillation:
     * - Phase 1: Drive to target position (no heading change)
     * - Phase 2: Rotate in place to target heading
     *
     * For small heading changes (≤30°), uses single-phase combined movement.
     *
     * @param targetX Target X position in FEET from field center
     * @param targetY Target Y position in FEET from field center
     * @param targetHeading Target heading in RADIANS
     */
    private void driveToPosition(double targetX, double targetY, double targetHeading) {
        targetX = Range.clip(targetX, FIELD_MIN_FEET, FIELD_MAX_FEET);
        targetY = Range.clip(targetY, FIELD_MIN_FEET, FIELD_MAX_FEET);

        // Two-phase threshold: heading changes larger than this use two-phase movement
        final double LARGE_HEADING_THRESHOLD_RAD = Math.toRadians(30.0);

        if (!odometryInitialized) {
            resetOdometryPods();
            odometryInitialized = true;
        }

        // Get current heading to determine if we need two-phase movement
        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double initialHeadingError = targetHeading - currentHeading;
        while (initialHeadingError > Math.PI) initialHeadingError -= 2 * Math.PI;
        while (initialHeadingError < -Math.PI) initialHeadingError += 2 * Math.PI;

        boolean useTwoPhase = Math.abs(initialHeadingError) > LARGE_HEADING_THRESHOLD_RAD;

        if (useTwoPhase) {
            // TWO-PHASE MOVEMENT: First drive to position, then rotate
            telemetry.addLine("Using TWO-PHASE movement (large heading change)");
            telemetry.addData("Heading change", "%.1f° > 30° threshold", 
                    Math.toDegrees(Math.abs(initialHeadingError)));
            telemetry.update();

            // Phase 1: Drive to position without heading change (70% of timeout)
            driveToPositionPhase(targetX, targetY, currentHeading, 5.6, true, "PHASE 1: DRIVING");

            // Phase 2: Rotate in place to target heading (30% of timeout)
            driveToPositionPhase(targetX, targetY, targetHeading, 2.4, false, "PHASE 2: ROTATING");

        } else {
            // SINGLE-PHASE MOVEMENT: Combined drive and rotate
            driveToPositionPhase(targetX, targetY, targetHeading, 8.0, false, "SINGLE-PHASE");
        }

        stopDriveMotors();
        sleep(100);
    }

    /**
     * Internal helper method that executes a single phase of movement for driveToPosition.
     * Can be configured for position-only (Phase 1) or combined position+heading.
     *
     * @param targetX Target X position in FEET
     * @param targetY Target Y position in FEET
     * @param targetHeading Target heading in RADIANS
     * @param timeout Maximum time for this phase
     * @param positionOnlyPhase If true, skip heading correction (Phase 1)
     * @param phaseName Name for telemetry display
     */
    private void driveToPositionPhase(double targetX, double targetY, double targetHeading,
                                       double timeout, boolean positionOnlyPhase, String phaseName) {
        double startTime = getRuntime();

        while (opModeIsActive() && (getRuntime() - startTime) < timeout) {
            // Update robot position from odometry
            //robotX = getXOdoInches() / 12;
            //robotY = getYOdoInches() / 12;
            updateDriveEncoderOdometry(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);


            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;
            double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            robotHeading = heading;

            // Calculate heading error
            double headingError = targetHeading - heading;
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            boolean positionReached = distanceToTarget * 12 < POSITION_TOLERANCE_INCHES;
            boolean headingReached = Math.abs(headingError) < Math.toRadians(ANGLE_TOLERANCE_DEGREES);

            // Check exit conditions based on phase type
            if (positionOnlyPhase) {
                // Phase 1: Exit when position is reached
                if (positionReached) {
                    stopDriveMotors();
                    break;
                }
            } else {
                // Phase 2 or Single-phase: Exit when both position AND heading are reached
                if (positionReached && headingReached) {
                    stopDriveMotors();
                    break;
                }
            }

            // Calculate drive powers
            double x = 0.0;
            double y = 0.0;
            double rx = 0.0;

            // Only drive if position not yet reached
            if (!positionReached) {
                double angleToTarget = Math.atan2(deltaX, deltaY);
                double speed = AUTO_MAX_SPEED;

                if (distanceToTarget < SLOWDOWN_DISTANCE_FEET) {
                    double slowdownRatio = distanceToTarget / SLOWDOWN_DISTANCE_FEET;
                    speed = AUTO_MIN_SPEED + (AUTO_MAX_SPEED - AUTO_MIN_SPEED) * slowdownRatio;
                }

                x = Math.sin(angleToTarget - heading) * speed;
                y = Math.cos(angleToTarget - heading) * speed;
            }

            // Heading correction (skip in position-only phase)
            if (!positionOnlyPhase && !headingReached) {
                // Scale down rotation when far from position to prevent oscillation (single-phase only)
                double rotationScale = 1.0;
                if (!positionReached && distanceToTarget > 2.0) { // 2 feet = ROTATION_SCALE_DIST equivalent
                    rotationScale = 0.2;
                } else if (!positionReached && distanceToTarget > POSITION_TOLERANCE_INCHES / 12.0) {
                    rotationScale = 0.2 + 0.8 * (1.0 - (distanceToTarget / 2.0));
                }

                rx = headingError * 0.5 * rotationScale;

                // Apply minimum rotation power when doing pure rotation (position reached)
                if (positionReached && Math.abs(rx) < 0.1 && !headingReached) {
                    rx = Math.signum(rx) * 0.1;
                }
            }

            driveFieldCentric(x, y, rx, heading);

            telemetry.addLine("--- " + phaseName + " ---");
            telemetry.addData("Target", "X: %.2f, Y: %.2f, H: %.1f°", targetX, targetY, Math.toDegrees(targetHeading));
            telemetry.addData("Current", "X: %.2f, Y: %.2f, H: %.1f°", robotX, robotY, Math.toDegrees(heading));
            telemetry.addData("Distance", "%.2f ft", distanceToTarget);
            telemetry.addData("Heading Error", "%.1f°", Math.toDegrees(headingError));
            telemetry.addData("Time remaining", "%.1f s", timeout - (getRuntime() - startTime));
            telemetry.update();
        }
    }

    private void resetOdometry() {
        // Reset encoder positions - use drive motors if no dedicated odometry pods
//        lastLeftEncoderPos = frontLeftMotor.getCurrentPosition();
//        lastRightEncoderPos = frontRightMotor.getCurrentPosition();
//        lastStrafeEncoderPos = backLeftMotor.getCurrentPosition();
        // ARATRIKAAAA
        odometryInitialized = true;
    }

    private void updateOdometryPosition() {
        // ARATRIKA YA GOT A LOTTA WORK TO DO </3
        // Get current encoder positions (using drive motors as odometry if no dedicated pods)
        int leftPos = frontLeftMotor.getCurrentPosition();
        int rightPos = frontRightMotor.getCurrentPosition();
        int strafePos = backLeftMotor.getCurrentPosition();

        // Calculate deltas
        int leftDelta = leftPos - lastLeftEncoderPos;
        int rightDelta = rightPos - lastRightEncoderPos;
        int strafeDelta = strafePos - lastStrafeEncoderPos;

        // Update last positions
        lastLeftEncoderPos = leftPos;
        lastRightEncoderPos = rightPos;
        lastStrafeEncoderPos = strafePos;

        // Convert ticks to inches
        double leftDist = leftDelta * ODOMETRY_INCHES_PER_TICK;
        double rightDist = rightDelta * ODOMETRY_INCHES_PER_TICK;
        double strafeDist = strafeDelta * ODOMETRY_INCHES_PER_TICK;

        // Calculate forward and strafe movement
        double forwardDist = (leftDist + rightDist) / 2.0;

        // Get current heading
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Convert robot-relative movement to field-relative
        double deltaXField = (forwardDist * Math.sin(heading) + strafeDist * Math.cos(heading)) / 12.0;
        double deltaYField = (forwardDist * Math.cos(heading) - strafeDist * Math.sin(heading)) / 12.0;

        // Update robot position (in feet)
        robotX += deltaXField;
        robotY += deltaYField;
        robotHeading = heading;

        // Clamp position to field bounds
        robotX = Range.clip(robotX, FIELD_MIN_FEET, FIELD_MAX_FEET);
        robotY = Range.clip(robotY, FIELD_MIN_FEET, FIELD_MAX_FEET);
    }

    private void stopDriveMotors () {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void stopAllMotors () {
        stopDriveMotors();
        intakeMotor.setPower(0);
        launchMotor.setPower(0);
    }

    private void driveFieldCentric ( double x, double y, double rx, double botHeading){
        // Field-centric transformation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Strafe correction

        // Calculate motor powers
        //double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        //double frontLeftPower = (rotY + rotX + rx) / denominator;
        //double backLeftPower = (rotY - rotX + rx) / denominator;
        //double frontRightPower = (rotY - rotX - rx) / denominator;
        //double backRightPower = (rotY + rotX - rx) / denominator;


        double frontLeftPower = -rotY + rotX + rx;
        double backLeftPower = -rotY - rotX + rx;  // Inverted y for proper strafe
        double frontRightPower = -rotY - rotX - rx;
        double backRightPower = -rotY + rotX - rx;  // Inverted y for proper strafe

        // Find the maximum absolute power to maintain proportional relationships
        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        // Scale down proportionally if any power exceeds maxDrivePower
        // This preserves the motor power ratios while respecting the power limit
        if (maxPower > 1.0) {
            double scale = 1.0 / maxPower;
            frontLeftPower *= scale;
            backLeftPower *= scale;
            frontRightPower *= scale;
            backRightPower *= scale;
        }

        // Clamp all motor powers to ±0.5 to ensure they never exceed the limit
        frontLeftPower = Range.clip(frontLeftPower, -0.5, 0.5);
        backLeftPower = Range.clip(backLeftPower, -0.5, 0.5);
        frontRightPower = Range.clip(frontRightPower, -0.5, 0.5);
        backRightPower = Range.clip(backRightPower, -0.5, 0.5);


        frontLeftMotor.setPower(frontLeftPower/2);
        backLeftMotor.setPower(backLeftPower/2);
        frontRightMotor.setPower(frontRightPower/2);
        backRightMotor.setPower(backRightPower/2);
    }

    // ========== ODOMETRY POD API ==========

    /**
     * Gets the raw encoder position from the X odometry pod.
     * Note: Value is negated to match motor connection direction.
     * @return Current encoder tick count from xOdo (negated)
     */
    public int getXOdoPosition() {
        return -xodo.getCurrentPosition();
    }

    /**
     * Gets the raw encoder position from the Y odometry pod.
     * @return Current encoder tick count from yOdo
     */
    public int getYOdoPosition() {
        return yodo.getCurrentPosition();
    }

    /**
     * Gets the X odometry pod position converted to inches.
     * Note: Value is negated to match motor connection direction.
     * @return X odometry position in inches
     */
    public double getXOdoInches() {
        return -xodo.getCurrentPosition() * ODOMETRY_INCHES_PER_TICK;
    }

    /**
     * Gets the Y odometry pod position converted to inches.
     * @return Y odometry position in inches
     */
    public double getYOdoInches() {
        return yodo.getCurrentPosition() * ODOMETRY_INCHES_PER_TICK;
    }

    /**
     * Resets both odometry pod encoders to zero.
     */
    public void resetOdometryPods() {
        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ========== ODOMETRY WHEEL NAVIGATION API ==========

    /**
     * Drives the robot to a specific position on the FTC 2026 field using xOdo and yOdo wheels.
     * Uses a PID-like control loop for smooth and accurate positioning.
     *
     * FTC 2026 Field Coordinates:
     * - Origin (0, 0) is at field center
     * - X-axis: positive toward red alliance wall, negative toward blue
     * - Y-axis: positive toward audience, negative toward scoring tables
     * - Field size: 12ft x 12ft (-6ft to +6ft on each axis)
     *
     * @param targetXInches Target X position in inches from field center
     * @param targetYInches Target Y position in inches from field center
     * @param targetHeadingDeg Target heading in degrees (0 = forward, positive = CCW)
     * @param maxSpeed Maximum drive speed (0.0 to 1.0)
     * @param timeoutSeconds Maximum time to reach target before giving up
     * @return true if position reached within tolerance, false if timed out
     */
    public boolean driveToPositionOdoWheels(double targetXInches, double targetYInches,
                                            double targetHeadingDeg, double maxSpeed,
                                            double timeoutSeconds) {
        // Field bounds in inches (12ft x 12ft field = 144in x 144in)
        final double FIELD_MIN_INCHES = -72.0;
        final double FIELD_MAX_INCHES = 72.0;

        // Position tolerances
        final double POSITION_TOLERANCE = 1.0; // inches
        final double HEADING_TOLERANCE = 2.0;  // degrees

        // PID-like control gains
        final double KP_DRIVE = 0.03;    // Proportional gain for position
        final double KP_HEADING = 0.02;  // Proportional gain for heading
        final double MIN_POWER = 0.15;   // Minimum power to overcome friction
        final double SLOWDOWN_DIST = 12.0; // Start slowing down at 12 inches

        // Clamp target to field bounds
        targetXInches = Range.clip(targetXInches, FIELD_MIN_INCHES, FIELD_MAX_INCHES);
        targetYInches = Range.clip(targetYInches, FIELD_MIN_INCHES, FIELD_MAX_INCHES);

        // Convert target heading to radians
        double targetHeadingRad = Math.toRadians(targetHeadingDeg);

        // Reset odometry pods to establish current position as origin
        resetOdometryPods();

        // Calculate initial position offsets from odometry
        double startXOdo = getXOdoInches();
        double startYOdo = getYOdoInches();

        double startTime = getRuntime();
        boolean targetReached = false;

        while (opModeIsActive() && (getRuntime() - startTime) < timeoutSeconds && !targetReached) {
            // Read current position from odometry wheels
            double currentXInches = getXOdoInches() - startXOdo;
            double currentYInches = getYOdoInches() - startYOdo;
            double currentHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double currentHeadingDeg = Math.toDegrees(currentHeadingRad);

            // Calculate position errors
            double errorX = targetXInches - currentXInches;
            double errorY = targetYInches - currentYInches;
            double distance = Math.sqrt(errorX * errorX + errorY * errorY);

            // Calculate heading error (normalized to -180 to 180)
            double headingError = targetHeadingDeg - currentHeadingDeg;
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            // Check if target reached
            if (distance < POSITION_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE) {
                targetReached = true;
                break;
            }

            // Calculate drive angle in field frame
            double angleToTarget = Math.atan2(errorX, errorY);

            // Speed scaling based on distance (slow down as we approach target)
            double speedScale = maxSpeed;
            if (distance < SLOWDOWN_DIST) {
                speedScale = MIN_POWER + (maxSpeed - MIN_POWER) * (distance / SLOWDOWN_DIST);
            }

            // Convert field-centric movement to robot-centric
            double robotAngle = angleToTarget - currentHeadingRad;
            double driveX = Math.sin(robotAngle) * speedScale * KP_DRIVE * distance;
            double driveY = Math.cos(robotAngle) * speedScale * KP_DRIVE * distance;

            // Clamp drive powers
            driveX = Range.clip(driveX, -maxSpeed, maxSpeed);
            driveY = Range.clip(driveY, -maxSpeed, maxSpeed);

            // Apply minimum power if needed to overcome friction
            if (distance > POSITION_TOLERANCE) {
                if (Math.abs(driveX) < MIN_POWER && Math.abs(driveX) > 0.01) {
                    driveX = Math.signum(driveX) * MIN_POWER;
                }
                if (Math.abs(driveY) < MIN_POWER && Math.abs(driveY) > 0.01) {
                    driveY = Math.signum(driveY) * MIN_POWER;
                }
            }

            // Heading correction
            double rotationPower = headingError * KP_HEADING;
            rotationPower = Range.clip(rotationPower, -maxSpeed * 0.5, maxSpeed * 0.5);

            // Calculate mecanum wheel powers
            double frontLeftPower = -driveY + driveX + rotationPower;
            double backLeftPower = -driveY - driveX + rotationPower;
            double frontRightPower = -driveY - driveX - rotationPower;
            double backRightPower = -driveY + driveX - rotationPower;

            // Normalize motor powers
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

            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower/2);
            backLeftMotor.setPower(backLeftPower/2);
            frontRightMotor.setPower(frontRightPower/2);
            backRightMotor.setPower(backRightPower/2);

            // Update telemetry
            telemetry.addLine("--- Odo Wheel Navigation ---");
            telemetry.addData("Target", "X: %.1f, Y: %.1f, H: %.1f°",
                    targetXInches, targetYInches, targetHeadingDeg);
            telemetry.addData("Current", "X: %.1f, Y: %.1f, H: %.1f°",
                    currentXInches, currentYInches, currentHeadingDeg);
            telemetry.addData("Distance", "%.2f in", distance);
            telemetry.addData("Heading Error", "%.1f°", headingError);
            telemetry.addData("xOdo Raw", "%d ticks", getXOdoPosition());
            telemetry.addData("yOdo Raw", "%d ticks", getYOdoPosition());
            telemetry.update();
        }

        // Stop all motors
        stopDriveMotors();

        return targetReached;
    }

    /**
     * Simplified version: drives to position with default speed and timeout.
     * @param targetXInches Target X position in inches from field center
     * @param targetYInches Target Y position in inches from field center
     * @return true if position reached, false if timed out
     */
    public boolean driveToPositionOdoWheels(double targetXInches, double targetYInches) {
        return driveToPositionOdoWheels(targetXInches, targetYInches, 0.0, 0.5, 5.0);
    }

    /**
     * Version with heading control and default speed/timeout.
     * @param targetXInches Target X position in inches from field center
     * @param targetYInches Target Y position in inches from field center
     * @param targetHeadingDeg Target heading in degrees
     * @return true if position reached, false if timed out
     */
    public boolean driveToPositionOdoWheels(double targetXInches, double targetYInches,
                                            double targetHeadingDeg) {
        return driveToPositionOdoWheels(targetXInches, targetYInches, targetHeadingDeg, 0.5, 5.0);
    }

    /**
     * Drives to a named field position for FTC 2026 DECODE season.
     * Predefined positions are relative to field center.
     *
     * @param positionName One of: "RED_LOWER_SPIKE", "RED_MIDDLE_SPIKE", "RED_TOP_SPIKE",
     *                     "RED_SHOOT", "OBELISK", "RED_START", "CENTER"
     * @return true if position reached, false if timed out or invalid position
     */
    public boolean driveToFieldPosition(String positionName) {
        double targetX, targetY;

        switch (positionName.toUpperCase()) {
            case "RED_LOWER_SPIKE":
                targetX = RED_LOWER_SPIKE_X * 12.0;  // Convert feet to inches
                targetY = RED_LOWER_SPIKE_Y * 12.0;
                break;
            case "RED_MIDDLE_SPIKE":
                targetX = RED_MIDDLE_SPIKE_X * 12.0;
                targetY = RED_MIDDLE_SPIKE_Y * 12.0;
                break;
            case "RED_TOP_SPIKE":
                targetX = RED_TOP_SPIKE_X * 12.0;
                targetY = RED_TOP_SPIKE_Y * 12.0;
                break;
            case "RED_SHOOT":
                targetX = RED_SHOOT_X * 12.0;
                targetY = RED_SHOOT_Y * 12.0;
                break;
            case "OBELISK":
                targetX = OBELISK_X * 12.0;
                targetY = OBELISK_Y * 12.0;
                break;
            case "RED_START":
                targetX = RED_DEFAULT_START_X * 12.0;
                targetY = RED_DEFAULT_START_Y * 12.0;
                break;
            case "CENTER":
                targetX = 0.0;
                targetY = 0.0;
                break;
            default:
                telemetry.addData("Error", "Unknown position: %s", positionName);
                telemetry.update();
                return false;
        }

        telemetry.addData("Navigating to", positionName);
        telemetry.addData("Target", "X: %.1f in, Y: %.1f in", targetX, targetY);
        telemetry.update();

        return driveToPositionOdoWheels(targetX, targetY);
    }

}

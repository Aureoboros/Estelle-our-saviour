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
    private static final double RED_SHOOT_Y = -4.0;        // Mirrored Y

    private static final double RED_PARK_X = -3.0;
    private static final double RED_PARK_Y = -3.0;

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
    public enum SpeedMode { SLOW, MEDIUM, FAST }
    private TeleOpTourney.SpeedMode currentSpeedMode = TeleOpTourney.SpeedMode.FAST;

    private double launchMotorPower = LAUNCH_MOTOR_POWER_HIGH;

    private int togglestopper = 1; // B toggle to open/close stopper, default close

    private int toggleintake = 1; // Y toggle intake default started

    private int togglespatula = 0; // toggle spatula, default down

    private int togglemotorpower = 1; // Back toggles launch motor power between high & low, default high for shooting from far

    private int tagid = 24; //red by default
    
    // Auto-alignment tracking
    private long lastAutoAlignTime = 0; // Track last auto-align timestamp
    private static final long AUTO_ALIGN_COOLDOWN_MS = 15000; // 15 second cooldown

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

        // Initialize drive encoder odometry reference values AFTER position detection
        // This ensures updateDriveEncoderOdometry() calculates deltas from current position
        // rather than accumulating from encoder position 0
        initializeDriveEncoderOdometry();

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
                driveToPosition(RED_SHOOT_X, RED_SHOOT_Y, 0);
            }
            if (bPressed) {
                //Park position
                if(tagid == 20) {
                    driveToPosition(-RED_PARK_X, RED_PARK_Y, 0);
                }
                else {
                    driveToPosition(RED_PARK_X, RED_PARK_Y, 0);
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

                // DPAD control for tag selection and motor power
                if (currentGamepad1.dpad_left) {
                    tagid = 20;
                    limelight.pipelineSwitch(1); // Switch to pipeline 1 for tag 20
                }
                else if (currentGamepad1.dpad_right) {
                    tagid = 24;
                    limelight.pipelineSwitch(0); // Switch to pipeline 0 for tag 24
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

                // DPAD control for tag selection and motor power
                if (currentGamepad2.dpad_left) {
                    tagid = 20;
                    limelight.pipelineSwitch(1); // Switch to pipeline 1 for tag 20
                }
                else if (currentGamepad2.dpad_right) {
                    tagid = 24;
                    limelight.pipelineSwitch(0); // Switch to pipeline 0 for tag 24
                }
                if (currentGamepad2.dpad_up) {
                    launchMotorPower += 0.05;
                }
                else if (currentGamepad2.dpad_down) {
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
        telemetry.addLine("Auto-aiming at target (Tag " + tagid + ")...");
        telemetry.update();

        boolean targetAcquired = false;
        double timeout = 3.0; // 3 second timeout
        double startTime = getRuntime();
        
        // Check if we're in cooldown period
        long currentTime = System.currentTimeMillis();
        boolean shouldAlignX = (currentTime - lastAutoAlignTime) >= AUTO_ALIGN_COOLDOWN_MS;
        
        if (!shouldAlignX) {
            long remainingCooldown = (AUTO_ALIGN_COOLDOWN_MS - (currentTime - lastAutoAlignTime)) / 1000;
            telemetry.addLine("X-axis alignment on cooldown");
            telemetry.addData("Time remaining", "%d seconds", remainingCooldown);
            telemetry.update();
        }

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

                        // Aim turret - with X-axis alignment control
                        aimTurretAtTag(fiducial, shouldAlignX);
                        
                        // Update last align time if we performed X alignment
                        if (shouldAlignX) {
                            lastAutoAlignTime = System.currentTimeMillis();
                        }

                        // Set launcher speed
                        launchMotor.setPower(launchMotorPower);
                        sleep(1000); // Allow launcher to spin up

                        // Launch sequence
                        launchBalls(1); // Launch 1 ball

                        targetAcquired = true;

                        telemetry.addData("Target Acquired", "Tag %d", tagid);
                        telemetry.addData("Distance (mm)", "%.0f", finalDistance);
                        telemetry.addData("Required RPM", "%.0f", requiredRPM);
                        if (shouldAlignX) {
                            telemetry.addLine("✓ X-axis aligned");
                        } else {
                            telemetry.addLine("○ X-axis alignment skipped (cooldown)");
                        }
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
            launchMotor.setPower(launchMotorPower);
            sleep(500);
            launchBalls(1);
        }
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

    private void aimTurretAtTag(LLResultTypes.FiducialResult fiducial, boolean alignX) {
        double tx = fiducial.getTargetXDegrees();

        // Turret tracking constants
        final double TX_TOLERANCE = 0.5;        // Tighter tolerance for dead-on alignment
        final double SERVO_MIN = 0.0;           
        final double SERVO_MAX = 0.5;           
        final double SERVO_CENTER = 0.25;       

        // Increased gain for more aggressive, accurate alignment
        final double BASE_GAIN = 0.008;         // Increased from 0.003
        final double PROPORTIONAL_GAIN = BASE_GAIN;
        final double MAX_STEP = 0.015;          // Slightly increased max step

        // Only align X-axis if allowed (first time or after cooldown)
        if (!alignX) {
            telemetry.addData("Turret", "SKIPPING X-ALIGN (cooldown active, TX: %.1f°)", tx);
            return;
        }

        // Check if already on target
        if (Math.abs(tx) <= TX_TOLERANCE) {
            telemetry.addData("Turret", "DEAD ON TARGET (TX: %.1f°)", tx);
            return;
        }

        // Multi-step alignment for dead-on accuracy
        int maxIterations = 10; // Allow multiple correction cycles
        int iteration = 0;
        
        while (Math.abs(tx) > TX_TOLERANCE && iteration < maxIterations && opModeIsActive()) {
            // Calculate adjustment
            double adjustment = Range.clip(-tx * PROPORTIONAL_GAIN, -MAX_STEP, MAX_STEP);

            double currentPos = spinSpinServo.getPosition();
            double newPos = currentPos + adjustment;

            // Clip to valid servo range
            newPos = Range.clip(newPos, SERVO_MIN, SERVO_MAX);
            spinSpinServo.setPosition(newPos);

            telemetry.addData("Turret Align", "Iteration %d/%d", iteration + 1, maxIterations);
            telemetry.addData("TX Error", "%.2f°", tx);
            telemetry.addData("Adjustment", "%.4f", adjustment);
            telemetry.addData("Position", "%.3f → %.3f", currentPos, newPos);
            telemetry.update();

            // Wait for servo to move and get fresh reading
            sleep(200);
            
            // Get fresh target reading
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid() && result.getFiducialResults().size() > 0) {
                for (LLResultTypes.FiducialResult fid : result.getFiducialResults()) {
                    if (fid.getFiducialId() == tagid) {
                        tx = fid.getTargetXDegrees();
                        break;
                    }
                }
            }
            
            iteration++;
        }
        
        if (Math.abs(tx) <= TX_TOLERANCE) {
            telemetry.addLine("✓ ACHIEVED DEAD-ON ALIGNMENT");
        } else {
            telemetry.addData("⚠ Alignment stopped", "TX: %.2f° (close enough)", tx);
        }
        telemetry.update();
        
        sleep(300); // Final settle time
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

    /**
     * Initializes drive encoder odometry by capturing current encoder positions as reference.
     * MUST be called after detectInitialPosition() and before the main loop starts.
     * This ensures that updateDriveEncoderOdometry() calculates deltas from the current position,
     * not from encoder position 0, which would corrupt the detected robotX/robotY values.
     */
    private void initializeDriveEncoderOdometry() {
        lastFLEncoder = frontLeftMotor.getCurrentPosition();
        lastBLEncoder = backLeftMotor.getCurrentPosition();
        lastFREncoder = frontRightMotor.getCurrentPosition();
        lastBREncoder = backRightMotor.getCurrentPosition();

        telemetry.addLine("Drive encoder odometry initialized");
        telemetry.addData("Initial robotX", "%.2f ft", robotX);
        telemetry.addData("Initial robotY", "%.2f ft", robotY);
        telemetry.addData("FL Encoder Reference", lastFLEncoder);
        telemetry.addData("BL Encoder Reference", lastBLEncoder);
        telemetry.addData("FR Encoder Reference", lastFREncoder);
        telemetry.addData("BR Encoder Reference", lastBREncoder);

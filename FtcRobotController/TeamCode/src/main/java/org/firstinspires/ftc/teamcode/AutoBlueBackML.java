package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="AutoBlueBackTest with MathLib", group="Autonomous")
public class AutoBlueBackML extends LinearOpMode {

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
    private static final int TARGET_APRILTAG_ID = 20;  // Blue alliance goal

    // DECODE Season AprilTag positions (in mm from field center)
    private static final double[][] TAG_POSITIONS = {
        {0.0, FIELD_SIZE_MM / 2},      // Tag 20 - Blue Alliance Goal
        {0.0, 1828.8},                 // Tag 21 - Neutral (0, 6ft)
        {0.0, 1828.8},                 // Tag 22 - Neutral (0, 6ft)
        {0.0, 1828.8},                 // Tag 23 - Neutral (0, 6ft)
        {0.0, -FIELD_SIZE_MM / 2}      // Tag 24 - Red Alliance Goal
    };

    // Field positions (in feet, measured from field center)
    private static final double BLUE_LOWER_SPIKE_X = -3.0;
    private static final double BLUE_LOWER_SPIKE_Y = -3.0;
    private static final double BLUE_MIDDLE_SPIKE_X = -3.0;
    private static final double BLUE_MIDDLE_SPIKE_Y = -1.0;
    private static final double BLUE_TOP_SPIKE_X = -3.0;
    private static final double BLUE_TOP_SPIKE_Y = 1.0;
    private static final double BLUE_SHOOT_X = 0.0;
    private static final double BLUE_SHOOT_Y = 4.0;
    private static final double OBELISK_X = 0.0;
    private static final double OBELISK_Y = 6.0;
    private static final double BLUE_DEFAULT_START_X = -1.0;
    private static final double BLUE_DEFAULT_START_Y = -5.0;
    private static final double FIELD_MIN_FEET = -6.0;
    private static final double FIELD_MAX_FEET = 6.0;

    // Odometry constants (goBILDA Odometry Pod with 35mm wheel)
    // Calculation: (π × 1.378 inches) / 2000 CPR = 0.002164 inches per tick
    private static final double ODOMETRY_INCHES_PER_TICK = 0.002164;
    private static final double COUNTS_PER_MM = 1.0; // CALIBRATE THIS

    // Tracking
    private double robotX = BLUE_DEFAULT_START_X;
    private double robotY = BLUE_DEFAULT_START_Y;
    private double robotHeading = 0.0;
    private boolean odometryInitialized = false;
    private int lastLeftEncoderPos = 0;
    private int lastRightEncoderPos = 0;
    private int lastStrafeEncoderPos = 0;
    private boolean positionDetected = false;

    // Absolute odometry tracking (for driveToPositionOdoWheels)
    private double absoluteFieldX = 0.0;  // Current X position in inches (absolute field coordinates)
    private double absoluteFieldY = 0.0;  // Current Y position in inches (absolute field coordinates)
    private double lastXOdoPosition = BLUE_DEFAULT_START_X * 12 / ODOMETRY_INCHES_PER_TICK;     // Last xOdo encoder reading
    private double lastYOdoPosition = BLUE_DEFAULT_START_Y * 12 / ODOMETRY_INCHES_PER_TICK;     // Last yOdo encoder reading
    private boolean absoluteOdometryInitialized = false;

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor, launchMotor;
    private DcMotor xodo, yodo;
    private Servo spinSpinServo, spatulaServo, stopServo;
    private IMU imu;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeHardware();

        telemetry.addLine("========================================");
        telemetry.addLine("BLUE ALLIANCE: GATE WEE WEE AUTO");
        telemetry.addLine("WITH LIMELIGHT AUTO-AIMING");
        telemetry.addLine("========================================");
        telemetry.addLine();

        // Try to detect initial position using Limelight
        detectInitialPosition();

        // Fallback if no tag detected
        if (!positionDetected) {
            robotX = BLUE_DEFAULT_START_X;
            robotY = BLUE_DEFAULT_START_Y;
            telemetry.addLine("⚠ WARNING: Could not detect position!");
            telemetry.addLine("Using default Blue start position");
            telemetry.addData("Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
            telemetry.addLine("Press START to continue anyway");
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        // Initialize absolute odometry with robot's starting position
        // This ensures odo wheels know the proper field position from the start
        double startXInches = robotX * 12.0;  // Convert feet to inches
        double startYInches = robotY * 12.0;
        initializeAbsoluteOdometry(startXInches, startYInches);
        
        telemetry.addData("Odometry Initialized", "X: %.1f in, Y: %.1f in", startXInches, startYInches);
        telemetry.update();

        // ========== SEQUENCE START ==========

        // 1. Shoot balls with auto-aim
        telemetry.addData("Step 1", "Navigate and Shoot Balls");
        telemetry.update();
        // driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        driveToPositionOdoWheels(BLUE_SHOOT_X * 12, BLUE_SHOOT_Y * 12);
        autoAimAndShoot();

        // 4. Intake gate, shoot balls, repeat
        // for (int i = 0; i < 4; i++) {
        //     telemetry.addData("Loop Iteration", i + 1);
        //     telemetry.update();
            
        //     // Intake gate
        //     telemetry.addData("Loop Step 1", "Intake Gate");
        //     telemetry.update();
        //     intakeGateBalls();

        //     // Shoot balls
        //     telemetry.addData("Loop Step 2", "Navigate and Shoot Balls");
        //     telemetry.update();
        //     driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        //     autoAimAndShoot();
        // }

        // 5. Intake top spike

        telemetry.addData("Step 2", "Intake Lower Spike");
        telemetry.update();
        intakeSpikeBalls(BLUE_LOWER_SPIKE_X * 12, BLUE_LOWER_SPIKE_Y * 12, 0);

        // 3. Shoot balls with auto-aim
        telemetry.addData("Step 3", "Navigate and Shoot Balls");
        telemetry.update();
        // driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        driveToPositionOdoWheels(BLUE_SHOOT_X * 12, BLUE_SHOOT_Y * 12);
        autoAimAndShoot();
        
        telemetry.addData("Step 4", "Intake Middle Spike");
        telemetry.update();
        intakeSpikeBalls(BLUE_MIDDLE_SPIKE_X * 12, BLUE_MIDDLE_SPIKE_Y * 12, 0);

        // 3. Shoot balls with auto-aim
        telemetry.addData("Step 5", "Navigate and Shoot Balls");
        telemetry.update();
        // driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        driveToPositionOdoWheels(BLUE_SHOOT_X * 12, BLUE_SHOOT_Y * 12);
        autoAimAndShoot();

        telemetry.addData("Step 6", "Intake Top Spike");
        telemetry.update();
        intakeSpikeBalls(BLUE_TOP_SPIKE_X * 12, BLUE_TOP_SPIKE_Y * 12, 0);

        telemetry.addData("Step 7", "Navigate and Shoot Balls");
        telemetry.update();
        // driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        driveToPositionOdoWheels(BLUE_SHOOT_X * 12, BLUE_SHOOT_Y * 12);
        autoAimAndShoot();
        
                // 2. Intake middle spike
        
        // Stop all motors
        stopAllMotors();

        telemetry.addLine("========================================");
        telemetry.addLine("AUTONOMOUS COMPLETE!");
        telemetry.addLine("========================================");
        telemetry.update();
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
        telemetry.addData("stop position", stopServo.getPosition());
        telemetry.update();


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
        
        // Verify Limelight connection
        telemetry.addLine("--- Limelight Status ---");
        telemetry.addData("Limelight Connected", limelight.isConnected());
        telemetry.addData("Limelight Running", limelight.isRunning());
        telemetry.update();
        sleep(500); // Give Limelight time to initialize

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servo positions
        stopServo.setPosition(1.0); // Closed
        spatulaServo.setPosition(0.0); // Down
    }

    private void detectInitialPosition() {
        telemetry.addLine("Detecting initial position...");
        telemetry.update();
        
        for (int i = 0; i < 30 && !positionDetected; i++) {
            LLResult result = limelight.getLatestResult();
            
            // Debug: Show raw Limelight data
            telemetry.addLine("--- Limelight Raw Data ---");
            telemetry.addData("Attempt", "%d/30", i + 1);
            telemetry.addData("Limelight Connected", limelight.isConnected());
            telemetry.addData("Limelight Running", limelight.isRunning());
            
            if (result == null) {
                telemetry.addData("Result", "NULL - No data from Limelight");
                telemetry.update();
                sleep(100);
                continue;
            }
            
            telemetry.addData("Result Valid", result.isValid());
            telemetry.addData("Pipeline Index", result.getPipelineIndex());
            telemetry.addData("Timestamp", "%.3f", result.getTimestamp());
            telemetry.addData("Targeting Latency", "%.1f ms", result.getTargetingLatency());
            
            // Show fiducial (AprilTag) results
            int fiducialCount = result.getFiducialResults().size();
            telemetry.addData("AprilTags Detected", fiducialCount);
            
            if (result.isValid() && fiducialCount > 0) {
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    int tagId = fiducial.getFiducialId();
                    
                    telemetry.addLine();
                    telemetry.addData("Tag ID", tagId);
                    telemetry.addData("TX (horizontal offset)", "%.2f°", fiducial.getTargetXDegrees());
                    telemetry.addData("TY (vertical offset)", "%.2f°", fiducial.getTargetYDegrees());
                    telemetry.addData("Target Area", "%.4f", fiducial.getTargetArea());
                    
                    // Use any visible tag to establish position
                    if (tagId >= 20 && tagId <= 24) {
                        double distance = calculateDistanceFromLimelight(fiducial);
                        double angle = Math.toRadians(fiducial.getTargetXDegrees());
                        
                        telemetry.addData("Calculated Distance", "%.1f mm", distance);
                        
                        // Calculate robot position relative to tag
                        double[] tagPos = TAG_POSITIONS[tagId - 20];
                        robotX = (tagPos[0] - distance * Math.sin(angle)) / 304.8; // mm to feet
                        robotY = (tagPos[1] - distance * Math.cos(angle)) / 304.8;
                        
                        positionDetected = true;
                        telemetry.addLine();
                        telemetry.addData("✓ Position Detected via Tag", tagId);
                        telemetry.addData("Robot Position", "X: %.2f ft, Y: %.2f ft", robotX, robotY);
                        telemetry.update();
                        sleep(1000); // Show result for 1 second
                        break;
                    }
                }
            } else {
                telemetry.addLine();
                telemetry.addData("Status", "No valid AprilTags in view");
            }
            
            telemetry.update();
            sleep(100);
        }
        
        if (!positionDetected) {
            telemetry.addLine("--- Position Detection Failed ---");
            telemetry.addData("Limelight Connected", limelight.isConnected());
            telemetry.addData("Limelight Running", limelight.isRunning());
            telemetry.addLine("Check: Camera view, AprilTag visibility, Pipeline settings");
            telemetry.update();
        }
    }

    private void autoAimAndShoot() {
        telemetry.addLine("Auto-aiming at target...");
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
                        
                        telemetry.addData("Target Acquired", "Tag %d", TARGET_APRILTAG_ID);
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
        
        double newPos = Range.clip(currentPos + angleOffset, 0.0, 0.2);
        spinSpinServo.setPosition(newPos);
        
        sleep(300); // Allow turret to settle
    }

    private void setLauncherSpeed(double rpm) {
        double power = Range.clip(rpm / MAX_MOTOR_RPM, 0.0, 1.0);
        launchMotor.setPower(power);
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
        //launchMotor.setPower(-LAUNCH_MOTOR_POWER);
        //sleep(2000);
        for (int i = 0; i < count; i++) {
            // Open stopper to allow ball through
            stopServo.setPosition(1.0);
            sleep(1000);
            // Close stopper to stop other balls from going under the spatula
            stopServo.setPosition(0.5);
            //while (stopServo.getPosition() != 0.0);
            sleep(1500);
            // Actuate spatula to push ball
            spatulaServo.setPosition(0.0);
            sleep(1000);
            spatulaServo.setPosition(1.0);
            sleep(1500);
            //while (spatulaServo.getPosition() != 1.0);
            // Close stopper
            //stopServo.setPosition(1.0);
            //sleep(200);
        }
    }

    private void driveToPosition(double targetX, double targetY, double targetHeading) {
        targetX = Range.clip(targetX, FIELD_MIN_FEET, FIELD_MAX_FEET);
        targetY = Range.clip(targetY, FIELD_MIN_FEET, FIELD_MAX_FEET);

        double startTime = getRuntime();
        double timeout = 8.0;

        if (!odometryInitialized) {
            resetOdometry();
        }

        while (opModeIsActive() && (getRuntime() - startTime) < timeout) {
            updateOdometryPosition();

            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;
            double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            if (distanceToTarget * 12 < POSITION_TOLERANCE_INCHES) {
                stopDriveMotors();
                break;
            }

            double angleToTarget = Math.atan2(deltaX, deltaY);
            double speed = AUTO_MAX_SPEED;
            
            if (distanceToTarget < SLOWDOWN_DISTANCE_FEET) {
                double slowdownRatio = distanceToTarget / SLOWDOWN_DISTANCE_FEET;
                speed = AUTO_MIN_SPEED + (AUTO_MAX_SPEED - AUTO_MIN_SPEED) * slowdownRatio;
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            robotHeading = heading;

            double x = Math.sin(angleToTarget - heading) * speed;
            double y = Math.cos(angleToTarget - heading) * speed;

            double headingError = targetHeading - heading;
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            double rx = headingError * 0.5;

            driveFieldCentric(x, y, rx, heading);

            telemetry.addData("Target", "X: %.2f, Y: %.2f", targetX, targetY);
            telemetry.addData("Current", "X: %.2f, Y: %.2f", robotX, robotY);
            telemetry.addData("Distance", "%.2f ft", distanceToTarget);
            telemetry.update();
        }

        stopDriveMotors();
        sleep(100);
    }

    private void resetOdometry() {
        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odometryInitialized = true;
    }

    private void updateOdometryPosition() {
        int leftPos = frontLeftMotor.getCurrentPosition();
        int rightPos = frontRightMotor.getCurrentPosition();
        int strafePos = backLeftMotor.getCurrentPosition();

        int leftDelta = leftPos - lastLeftEncoderPos;
        int rightDelta = rightPos - lastRightEncoderPos;
        int strafeDelta = strafePos - lastStrafeEncoderPos;

        lastLeftEncoderPos = leftPos;
        lastRightEncoderPos = rightPos;
        lastStrafeEncoderPos = strafePos;

        double leftDist = leftDelta * ODOMETRY_INCHES_PER_TICK;
        double rightDist = rightDelta * ODOMETRY_INCHES_PER_TICK;
        double strafeDist = strafeDelta * ODOMETRY_INCHES_PER_TICK;

        double forwardDist = (leftDist + rightDist) / 2.0;
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double deltaXField = (forwardDist * Math.sin(heading) + strafeDist * Math.cos(heading)) / 12.0;
        double deltaYField = (forwardDist * Math.cos(heading) - strafeDist * Math.sin(heading)) / 12.0;

        robotX += deltaXField;
        robotY += deltaYField;
        robotHeading = heading;

        robotX = Range.clip(robotX, FIELD_MIN_FEET, FIELD_MAX_FEET);
        robotY = Range.clip(robotY, FIELD_MIN_FEET, FIELD_MAX_FEET);
    }

    private void driveFieldCentric(double x, double y, double rx, double botHeading) {
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double frontLeftPower = -rotY + rotX + rx;
        double backLeftPower = -rotY - rotX + rx;
        double frontRightPower = -rotY - rotX - rx;
        double backRightPower = -rotY + rotX - rx;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            double scale = 1.0 / maxPower;
            frontLeftPower *= scale;
            backLeftPower *= scale;
            frontRightPower *= scale;
            backRightPower *= scale;
        }

        frontLeftPower = Range.clip(frontLeftPower, -0.5, 0.5);
        backLeftPower = Range.clip(backLeftPower, -0.5, 0.5);
        frontRightPower = Range.clip(frontRightPower, -0.5, 0.5);
        backRightPower = Range.clip(backRightPower, -0.5, 0.5);

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void stopDriveMotors() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void stopAllMotors() {
        stopDriveMotors();
        intakeMotor.setPower(0);
        launchMotor.setPower(0);
    }

    private void intakeGateBalls() {
        // Note: This method references undefined variables/methods from original code
        // You'll need to implement proper turn and duration logic
        double angle = Math.toRadians(60);
        turnToAngle(angle);
        // driveToPosition(-2.0, -1.0, angle);
        driveToPosition(-2.0, -1.0, angle);
        // driveToPosition(-5.0, -1.0, angle);
        driveToPosition(-5.0, -1.0, angle);
        intakeMotor.setPower(INTAKE_POWER);
        sleep(4000); // 4 second intake
        intakeMotor.setPower(0);
        // driveToPosition(-2.0, -1.0, angle);
        driveToPosition(-2.0, -1.0, angle);
    }

    private void intakeSpikeBalls(double spike_x, double spike_y, double angle) {
        // spike_x and spike_y are already in INCHES (converted at call site)
        // angle is in DEGREES
        driveToPositionOdoWheels(spike_x, spike_y, 0);
        turnToAngle(Math.toRadians(-angle)); // turnToAngle expects radians
        intakeMotor.setPower(INTAKE_POWER);
        // Move 24 inches (2 feet) in the -X direction for intake
        driveToPositionOdoWheels(spike_x - 24.0, spike_y, -angle); // driveToPositionOdoWheels expects degrees
        intakeMotor.setPower(0);
        driveToPositionOdoWheels(spike_x, spike_y, 0);
    }

    private void turnToAngle(double targetAngle) {
        double startTime = getRuntime();
        double timeout = 3.0;
        
        while (opModeIsActive() && (getRuntime() - startTime) < timeout) {
            double currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double error = targetAngle - currentAngle;
            
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;
            
            if (Math.abs(error) < Math.toRadians(ANGLE_TOLERANCE_DEGREES)) {
                break;
            }
            
            double turnPower = Range.clip(error * 0.5, -0.3, 0.3);
            
            frontLeftMotor.setPower(turnPower);
            backLeftMotor.setPower(turnPower);
            frontRightMotor.setPower(-turnPower);
            backRightMotor.setPower(-turnPower);
        }
        
        stopDriveMotors();
        sleep(100);
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
     * Initializes the absolute odometry system with the robot's starting position.
     * Call this once at the start of autonomous with the robot's known field position.
     * 
     * @param startXInches Starting X position in inches from field center
     * @param startYInches Starting Y position in inches from field center
     */
    public void initializeAbsoluteOdometry(double startXInches, double startYInches) {
        // Reset the odometry encoders
        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set the absolute position to the known starting position
        absoluteFieldX = startXInches;
        absoluteFieldY = startYInches;
        
        // Store current encoder positions as reference (use -ve for xodo, consistent with getXOdoInches)
        lastXOdoPosition = -absoluteFieldX * 12 / ODOMETRY_INCHES_PER_TICK;
        lastYOdoPosition = absoluteFieldY * 12 / ODOMETRY_INCHES_PER_TICK;
        absoluteOdometryInitialized = true;
        
        telemetry.addData("Odometry Initialized", "X: %.1f in, Y: %.1f in", absoluteFieldX, absoluteFieldY);
        telemetry.update();
    }

    /**
     * Updates the robot's absolute field position based on odometry wheel movements.
     * Call this continuously during autonomous to track position.
     */
    private void updateAbsoluteOdometry() {
        // Get current encoder positions (use -ve for xodo, consistent with getXOdoInches)
        int currentXOdo = -xodo.getCurrentPosition();
        int currentYOdo = yodo.getCurrentPosition();
        
        // Calculate deltas since last update
        double deltaXTicks = currentXOdo - lastXOdoPosition;
        double deltaYTicks = currentYOdo - lastYOdoPosition;
        
        // Convert ticks to inches
        double deltaXInches = deltaXTicks * ODOMETRY_INCHES_PER_TICK;
        double deltaYInches = deltaYTicks * ODOMETRY_INCHES_PER_TICK;
        
        // Get current heading for field-centric conversion
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        // Convert robot-relative movement to field-relative movement
        double fieldDeltaX = deltaXInches * Math.cos(heading) - deltaYInches * Math.sin(heading);
        double fieldDeltaY = deltaXInches * Math.sin(heading) + deltaYInches * Math.cos(heading);
        
        // Update absolute position
        absoluteFieldX += fieldDeltaX;
        absoluteFieldY += fieldDeltaY;
        
        // Store current encoder positions for next update
        lastXOdoPosition = currentXOdo;
        lastYOdoPosition = currentYOdo;
    }

    /**
     * Gets the current absolute X position on the field.
     * @return X position in inches from field center
     */
    public double getAbsoluteFieldX() {
        return absoluteFieldX;
    }

    /**
     * Gets the current absolute Y position on the field.
     * @return Y position in inches from field center
     */
    public double getAbsoluteFieldY() {
        return absoluteFieldY;
    }

    /**
     * Drives the robot to a specific ABSOLUTE position on the FTC 2026 field using xOdo and yOdo wheels.
     * Uses a PID-like control loop for smooth and accurate positioning.
     * 
     * IMPORTANT: Call initializeAbsoluteOdometry() once at the start of autonomous before using this method.
     *
     * FTC 2026 Field Coordinates:
     * - Origin (0, 0) is at field center
     * - X-axis: positive toward red alliance wall, negative toward blue
     * - Y-axis: positive toward audience, negative toward scoring tables
     * - Field size: 12ft x 12ft (-72in to +72in on each axis)
     *
     * @param targetXInches Target X position in inches from field center (ABSOLUTE)
     * @param targetYInches Target Y position in inches from field center (ABSOLUTE)
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
        
        // Anti-oscillation: scale down rotation when far from target position
        final double ROTATION_SCALE_DIST = 24.0; // Full rotation only within this distance

        // Clamp target to field bounds
        targetXInches = Range.clip(targetXInches, FIELD_MIN_INCHES, FIELD_MAX_INCHES);
        targetYInches = Range.clip(targetYInches, FIELD_MIN_INCHES, FIELD_MAX_INCHES);

        // Initialize absolute odometry if not already done (use current position as starting point)
        if (!absoluteOdometryInitialized) {
            // Default to BLUE_START if not initialized
            initializeAbsoluteOdometry(BLUE_DEFAULT_START_X * 12.0, BLUE_DEFAULT_START_Y * 12.0);
        }

        double startTime = getRuntime();
        boolean targetReached = false;

        while (opModeIsActive() && (getRuntime() - startTime) < timeoutSeconds && !targetReached) {
            // Update absolute position from odometry
            updateAbsoluteOdometry();
            
            // Get current absolute position
            double currentXInches = absoluteFieldX;
            double currentYInches = absoluteFieldY;
            double currentHeadingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double currentHeadingDeg = Math.toDegrees(currentHeadingRad);

            // Calculate position errors (ABSOLUTE: target - current)
            double errorX = targetXInches - currentXInches;
            double errorY = targetYInches - currentYInches;
            double distance = Math.sqrt(errorX * errorX + errorY * errorY);

            // Calculate heading error (normalized to -180 to 180)
            double headingError = targetHeadingDeg - currentHeadingDeg;
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            // Check if target reached (both position AND heading)
            boolean positionReached = distance < POSITION_TOLERANCE;
            boolean headingReached = Math.abs(headingError) < HEADING_TOLERANCE;
            
            if (positionReached && headingReached) {
                targetReached = true;
                break;
            }

            // Calculate drive powers
            double driveX = 0.0;
            double driveY = 0.0;
            double rotationPower = 0.0;
            
            // Only drive if position not yet reached
            if (!positionReached) {
                // Calculate drive angle in field frame
                double angleToTarget = Math.atan2(errorX, errorY);

                // Speed scaling based on distance (slow down as we approach target)
                double speedScale = maxSpeed;
                if (distance < SLOWDOWN_DIST) {
                    speedScale = MIN_POWER + (maxSpeed - MIN_POWER) * (distance / SLOWDOWN_DIST);
                }

                // Convert field-centric movement to robot-centric
                double robotAngle = angleToTarget - currentHeadingRad;
                driveX = Math.sin(robotAngle) * KP_DRIVE * distance; // removed speedScale bc distance^2
                driveY = Math.cos(robotAngle) * KP_DRIVE * distance; // same thing here

                // Clamp drive powers
                driveX = Range.clip(driveX, -maxSpeed, maxSpeed);
                driveY = Range.clip(driveY, -maxSpeed, maxSpeed);

                // Apply minimum power to overcome friction (only when driving)
                if (Math.abs(driveX) < MIN_POWER && Math.abs(driveX) > 0.01) {
                    driveX = Math.signum(driveX) * MIN_POWER;
                }
                if (Math.abs(driveY) < MIN_POWER && Math.abs(driveY) > 0.01) {
                    driveY = Math.signum(driveY) * MIN_POWER;
                }
            }

            // Heading correction - scale down when far from position to prevent oscillation
            if (!headingReached) {
                // Calculate rotation scale factor (0 to 1) based on distance
                // Full rotation when close to target, reduced when far
                double rotationScale = 1.0;
                if (distance > ROTATION_SCALE_DIST) {
                    // When far from target, reduce rotation to 20% to prevent oscillation
                    rotationScale = 0.2;
                } else if (distance > POSITION_TOLERANCE) {
                    // Linear interpolation between 20% and 100%
                    rotationScale = 0.2 + 0.8 * (1.0 - (distance / ROTATION_SCALE_DIST));
                }
                
                rotationPower = headingError * KP_HEADING * rotationScale;
                rotationPower = Range.clip(rotationPower, -maxSpeed * 0.5, maxSpeed * 0.5);
                
                // Apply minimum rotation power only when position is reached (pure rotation)
                if (positionReached && Math.abs(rotationPower) < MIN_POWER && Math.abs(headingError) > HEADING_TOLERANCE) {
                    rotationPower = Math.signum(rotationPower) * MIN_POWER;
                }
            }

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
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Update telemetry
            telemetry.addLine("--- Absolute Odo Navigation ---");
            telemetry.addData("Target (absolute)", "X: %.1f, Y: %.1f, H: %.1f°",
                    targetXInches, targetYInches, targetHeadingDeg);
            telemetry.addData("Current (absolute)", "X: %.1f, Y: %.1f, H: %.1f°",
                    currentXInches, currentYInches, currentHeadingDeg);
            telemetry.addData("Error", "X: %.1f, Y: %.1f", errorX, errorY);
            telemetry.addData("Distance to target", "%.2f in", distance);
            telemetry.addData("Heading Error", "%.1f°", headingError);
            telemetry.addData("Phase", positionReached ? "ROTATING" : "DRIVING");
            telemetry.addData("xOdo Raw", "%d ticks", getXOdoPosition());
            telemetry.addData("yOdo Raw", "%d ticks", getYOdoPosition());
            telemetry.update();
        }

        // Stop all motors
        stopDriveMotors();

        return targetReached;
    }

    /**
     * Simplified version: drives to ABSOLUTE position with default speed and dynamic timeout.
     * Timeout is calculated based on distance to allow sufficient travel time.
     * @param targetXInches Target X position in inches from field center (ABSOLUTE)
     * @param targetYInches Target Y position in inches from field center (ABSOLUTE)
     * @return true if position reached, false if timed out
     */
    public boolean driveToPositionOdoWheels(double targetXInches, double targetYInches) {
        // Calculate distance from current position to target for timeout calculation
        if (!absoluteOdometryInitialized) {
            initializeAbsoluteOdometry(BLUE_DEFAULT_START_X * 12.0, BLUE_DEFAULT_START_Y * 12.0);
        }
        
        double deltaX = targetXInches - absoluteFieldX;
        double deltaY = targetYInches - absoluteFieldY;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        
        // Estimate ~15 inches per second at 0.5 speed, add 3 seconds buffer
        double timeout = Math.max(5.0, (distance / 15.0) + 3.0);
        return driveToPositionOdoWheels(targetXInches, targetYInches, 0.0, 0.5, timeout);
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
     * @param positionName One of: "BLUE_LOWER_SPIKE", "BLUE_MIDDLE_SPIKE", "BLUE_TOP_SPIKE",
     *                     "BLUE_SHOOT", "OBELISK", "BLUE_START", "CENTER"
     * @return true if position reached, false if timed out or invalid position
     */
    public boolean driveToFieldPosition(String positionName) {
        double targetX, targetY;

        switch (positionName.toUpperCase()) {
            case "BLUE_LOWER_SPIKE":
                targetX = BLUE_LOWER_SPIKE_X * 12.0;  // Convert feet to inches
                targetY = BLUE_LOWER_SPIKE_Y * 12.0;
                break;
            case "BLUE_MIDDLE_SPIKE":
                targetX = BLUE_MIDDLE_SPIKE_X * 12.0;
                targetY = BLUE_MIDDLE_SPIKE_Y * 12.0;
                break;
            case "BLUE_TOP_SPIKE":
                targetX = BLUE_TOP_SPIKE_X * 12.0;
                targetY = BLUE_TOP_SPIKE_Y * 12.0;
                break;
            case "BLUE_SHOOT":
                targetX = BLUE_SHOOT_X * 12.0;
                targetY = BLUE_SHOOT_Y * 12.0;
                break;
            case "OBELISK":
                targetX = OBELISK_X * 12.0;
                targetY = OBELISK_Y * 12.0;
                break;
            case "BLUE_START":
                targetX = BLUE_DEFAULT_START_X * 12.0;
                targetY = BLUE_DEFAULT_START_Y * 12.0;
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

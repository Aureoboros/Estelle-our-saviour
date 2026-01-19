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

    // Odometry constants
    private static final double ODOMETRY_INCHES_PER_TICK = 0.002; // CALIBRATE THIS
    private static final double COUNTS_PER_MM = 19.894; // CALIBRATE THIS

    // Tracking
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;
    private boolean odometryInitialized = false;
    private int lastLeftEncoderPos = 0;
    private int lastRightEncoderPos = 0;
    private int lastStrafeEncoderPos = 0;
    private boolean positionDetected = false;

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

        // ========== SEQUENCE START ==========

        // 1. Shoot balls with auto-aim
        telemetry.addData("Step 1", "Navigate and Shoot Balls");
        telemetry.update();
        driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
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
        intakeSpikeBalls(BLUE_LOWER_SPIKE_X, BLUE_LOWER_SPIKE_Y, 0);

        // 3. Shoot balls with auto-aim
        telemetry.addData("Step 3", "Navigate and Shoot Balls");
        telemetry.update();
        driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        autoAimAndShoot();
        
        telemetry.addData("Step 4", "Intake Middle Spike");
        telemetry.update();
        intakeSpikeBalls(BLUE_MIDDLE_SPIKE_X, BLUE_MIDDLE_SPIKE_Y, 0);

        // 3. Shoot balls with auto-aim
        telemetry.addData("Step 5", "Navigate and Shoot Balls");
        telemetry.update();
        driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        autoAimAndShoot();

        telemetry.addData("Step 6", "Intake Top Spike");
        telemetry.update();
        intakeSpikeBalls(BLUE_TOP_SPIKE_X, BLUE_TOP_SPIKE_Y, 0);

        telemetry.addData("Step 7", "Navigate and Shoot Balls");
        telemetry.update();
        driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
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
        driveToPosition(-2.0, -1.0, angle);
        driveToPosition(-5.0, -1.0, angle);
        intakeMotor.setPower(INTAKE_POWER);
        sleep(4000); // 4 second intake
        intakeMotor.setPower(0);
        driveToPosition(-2.0, -1.0, angle);
    }

    private void intakeSpikeBalls(double spike_x, double spike_y, double angle) {
        driveToPosition(spike_x, spike_y, 0);
        turnToAngle(-Math.toRadians(angle));
        intakeMotor.setPower(INTAKE_POWER);
        driveToPosition(spike_x - 2.0, spike_y, -Math.toRadians(angle));
        intakeMotor.setPower(0);
        driveToPosition(spike_x, spike_y, 0);
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
}

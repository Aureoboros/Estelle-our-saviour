package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
@TeleOp(name = "TeleOp")
public class TeleOp extends LinearOpMode {

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

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor, launchMotor;
    private DcMotor xodo, yodo;
    private Servo spinSpinServo, spatulaServo, stopServo;
    private IMU imu;
    private Limelight3A limelight;


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

}
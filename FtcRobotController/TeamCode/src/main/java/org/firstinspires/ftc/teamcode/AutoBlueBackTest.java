// Deleted almost everything April-Tag related in the declaration stage

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name="AutoBlueBackTest", group="Autonomous")
public class AutoBlueBackTest extends LinearOpMode {

    // Motor power constants
    private static final double INTAKE_POWER = 1.0;
    private static final double LAUNCH_MOTOR_POWER = 0.7;  // Reduced for better accuracy
    private static final double SPATULA_SERVO_POWER = 0.8;

    // Navigation constants
    private static final double AUTO_MAX_SPEED = 0.7;
    private static final double AUTO_MIN_SPEED = 0.15;
    private static final double SLOWDOWN_DISTANCE_FEET = 2.0;
    private static final double POSITION_TOLERANCE_INCHES = 3.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.5;

    // AprilTag alignment constants
    private static final double APRILTAG_ALIGNMENT_TOLERANCE = 3.0;  // degrees
    private static final double APRILTAG_ROTATION_POWER = 0.25;
    private static final int TARGET_APRILTAG_ID = 20;  // Blue alliance goal

    // AprilTag IDs
    private static final int[] OBELISK_APRILTAG_IDS = {21, 22, 23};  // Obelisk tags for positioning

    // Field positions (in feet, measured from field center)
    // Blue alliance spike marks (left side of field, mirrored from red)
    private static final double BLUE_LOWER_SPIKE_X = -3.0;
    private static final double BLUE_LOWER_SPIKE_Y = -3.0;

    private static final double BLUE_MIDDLE_SPIKE_X = -3.0;  // Middle spike mark (mirrored)
    private static final double BLUE_MIDDLE_SPIKE_Y = -1.0;  // Below center line

    private static final double BLUE_TOP_SPIKE_X = -3.0;     // Top spike mark (same X)
    private static final double BLUE_TOP_SPIKE_Y = 1.0;      // Above center line

    // Shooting position (above field center for collision avoidance)
    private static final double BLUE_SHOOT_X = 0.0;
    private static final double BLUE_SHOOT_Y = 4.0;

    // Park position (blue observation zone - mirrored)
//    private static final double BLUE_PARK_X = 3.0;
//    private static final double BLUE_PARK_Y = -4.0;

    // Obelisk position (center structure with AprilTags)
    private static final double OBELISK_X = 0.0;
    private static final double OBELISK_Y = 6.0;

    // Default starting position if AprilTag detection fails
    private static final double BLUE_DEFAULT_START_X = -1.0;
    private static final double BLUE_DEFAULT_START_Y = -5.0;

    // Tracking (will be set based on actual start position)
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    // Hardware
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor, launchMotor, rampMotor;
    private IMU imu;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeHardware();

        telemetry.addLine("========================================");
        telemetry.addLine("BLUE ALLIANCE: GATE WEE WEE AUTO");
        telemetry.addLine("========================================");
        telemetry.addLine();

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

        // 1. Shoot balls
        telemetry.addData("Step 1", "Shoot Balls");
        telemetry.update();
        driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);

        // 2. Intake middle spike
        telemetry.addData("Step 2", "Intake Middle Spike");
        telemetry.update();
        intakeSpikeBalls(BLUE_MIDDLE_SPIKE_X, BLUE_MIDDLE_SPIKE_Y, 0);

        // 3. Shoot balls
        telemetry.addData("Step 3", "Shoot Balls");
        telemetry.update();
        driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);

        // 4. Intake gate, shoot balls, repeat
        for (int i = 0; i < 4; i++) {
            // 1. Intake gate
            telemetry.addData("Loop Step 1", "Intake Gate");
            telemetry.update();
            intakeGateBalls();

            // 2. Shoot balls
            telemetry.addData("Loop Step 2", "Shoot Balls");
            telemetry.update();
            driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);
        }

        // 5. Intake top spike
        telemetry.addData("Step 5", "Intake Top Spike");
        telemetry.update();
        intakeSpikeBalls(BLUE_TOP_SPIKE_X, BLUE_TOP_SPIKE_Y, 0);

        // 6. Shoot balls
        telemetry.addData("Step 6", "Shoot Balls");
        telemetry.update();
        driveToPosition(BLUE_SHOOT_X, BLUE_SHOOT_Y, 0);

        // Stop all motors
        stopAllMotors();

        telemetry.addLine("========================================");
        telemetry.addLine("AUTONOMOUS COMPLETE!");
        telemetry.addLine("========================================");
        telemetry.update();
    }

    private void driveToPosition(double targetX, double targetY, double targetHeading) {
        // Validate target is within FTC 2026 field bounds
        targetX = Range.clip(targetX, FIELD_MIN_FEET, FIELD_MAX_FEET);
        targetY = Range.clip(targetY, FIELD_MIN_FEET, FIELD_MAX_FEET);

        double startTime = getRuntime();
        double timeout = 8.0;  // 8 second timeout for longer distances

        // Initialize odometry if not already done
        if (!odometryInitialized) {
            resetOdometry();
        }

        while (opModeIsActive() && (getRuntime() - startTime) < timeout) {
            // Update robot position from odometry
            updateOdometryPosition();

            double deltaX = targetX - robotX;
            double deltaY = targetY - robotY;
            double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            // Check if reached target
            if (distanceToTarget * 12 < POSITION_TOLERANCE_INCHES) {
                stopDriveMotors();
                break;
            }

            // Calculate angle to target (field-centric)
            double angleToTarget = Math.atan2(deltaX, deltaY);

            // Speed control with slowdown near target
            double speed = AUTO_MAX_SPEED;
            if (distanceToTarget < SLOWDOWN_DISTANCE_FEET) {
                double slowdownRatio = distanceToTarget / SLOWDOWN_DISTANCE_FEET;
                speed = AUTO_MIN_SPEED + (AUTO_MAX_SPEED - AUTO_MIN_SPEED) * slowdownRatio;
            }

            // Get current heading from IMU
            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            robotHeading = heading;

            // Convert to robot frame
            double x = Math.sin(angleToTarget - heading) * speed;
            double y = Math.cos(angleToTarget - heading) * speed;

            // Calculate heading correction
            double headingError = targetHeading - heading;
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            double rx = headingError * 0.5;  // Proportional heading control

            // Drive
            driveFieldCentric(x, y, rx, heading);

            // Telemetry
            telemetry.addData("Target", "X: %.2f, Y: %.2f", targetX, targetY);
            telemetry.addData("Current", "X: %.2f, Y: %.2f", robotX, robotY);
            telemetry.addData("Distance", "%.2f ft", distanceToTarget);
            telemetry.addData("Speed", "%.2f", speed);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(heading));
            telemetry.update();
        }

        stopDriveMotors();
        sleep(100);  // Brief settle time
    }

    private void resetOdometry() {
        // Reset encoder positions - use drive motors if no dedicated odometry pods
        lastLeftEncoderPos = frontLeftMotor.getCurrentPosition();
        lastRightEncoderPos = frontRightMotor.getCurrentPosition();
        lastStrafeEncoderPos = backLeftMotor.getCurrentPosition();
        odometryInitialized = true;
    }

    private void updateOdometryPosition() {
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


        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
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
        rampMotor.setPower(0);
    }

    private void intakeGateBalls() {
        drive.turn(Math.toRadians(60));
        driveToPosition(-2.0, -1.0);
        driveToPosition(-5.0, 1.0); // OG values: (-5.5, -0.75)
//        intakeMotor.setPower(INTAKE_POWER);
        sleep((long)(duration * 4000));
//        intakeMotor.setPower(0);
      }

    private void intakeSpikeBalls(double spike_x, double spike_y, double angle) {
        // SOMEBODY NEEDS TO CHECK IF SPIKE_X AND Y ARE IN INCHES OR SOMETHING ELSE
        driveToPosition(spike_x, spike_y);
        drive.turn(-Math.toRadians(angle)); // CHECK PLEASE
//        intakeMotor.setPower(INTAKE_POWER);
        driveToPosition(spike_x - 2.0, spike_y);
//        intakeMotor.setPower(0);
        driveToPosition(spike_x, spike_y);
      }
}

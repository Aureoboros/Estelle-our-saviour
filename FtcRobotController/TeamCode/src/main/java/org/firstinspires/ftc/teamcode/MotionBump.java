package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Motion Bump Mode")
public class MotionBump extends LinearOpMode {

    // ========== MOTOR POWER CONSTANTS ==========
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;

    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 1.0;
    private static final double GAMEPAD2_MAX_POWER = 0.5;

    // Speed presets
    private static final double SPEED_SLOW = 0.3;
    private static final double SPEED_MEDIUM = 0.6;
    private static final double SPEED_FAST = 1.0;

    // ========== ACTIVE RESISTANCE CONSTANTS ==========
    private static final double VELOCITY_TRACK_KP = 0.25;      // Velocity tracking gain
    private static final double POSITION_HOLD_KP = 0.18;       // Position hold gain
    private static final double ROTATION_HOLD_KP = 0.9;        // Rotation hold gain
    private static final double VELOCITY_RESISTANCE_GAIN = 0.4; // How much to resist unwanted velocity
    private static final double MAX_RESISTANCE_POWER = 0.7;    // Maximum resistance power
    private static final double VELOCITY_DEADZONE = 1.5;       // inches/sec - minimum velocity to track
    private static final double POSITION_DEADZONE = 0.4;       // inches
    private static final double ROTATION_DEADZONE = 1.5;       // degrees

    // ========== DRIVE ENCODER ODOMETRY CONSTANTS ==========
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    // ========== STATE VARIABLES ==========
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;

    private int lastFLEncoder = 0;
    private int lastBLEncoder = 0;
    private int lastFREncoder = 0;
    private int lastBREncoder = 0;

    // Velocity tracking
    private double lastRobotX = 0.0;
    private double lastRobotY = 0.0;
    private double lastRobotHeading = 0.0;
    private long lastUpdateTime = 0;

    // Target tracking
    private double targetVelocityX = 0.0;  // inches/sec
    private double targetVelocityY = 0.0;
    private double targetRotationRate = 0.0;  // radians/sec
    private double holdX = 0.0;
    private double holdY = 0.0;
    private double holdHeading = 0.0;

    // Speed preset state
    private enum SpeedMode { SLOW, MEDIUM, FAST }
    private SpeedMode currentSpeedMode = SpeedMode.FAST;

    @Override
    public void runOpMode() throws InterruptedException {
        // ========== HARDWARE INITIALIZATION ==========

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // ========== INIT TELEMETRY ==========
        telemetry.addLine("========================================");
        telemetry.addLine("DRIVE MODE - ACTIVE RESISTANCE");
        telemetry.addLine("========================================");
        telemetry.addData("Drive Motors", "✓ (4 motors)");
        telemetry.addData("Odometry", "✓ (Drive Encoders)");
        telemetry.addData("IMU", "✓");
        telemetry.addData("Active Resistance", "✓ ENABLED");
        telemetry.addLine("========================================");
        telemetry.addLine("Resists ANY motion opposing driver intent:");
        telemetry.addLine("  - Holds position when stopped");
        telemetry.addLine("  - Maintains velocity when moving");
        telemetry.addLine("========================================");
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick Y = Forward/Back");
        telemetry.addLine("  Right Stick X = Strafe L/R");
        telemetry.addLine("  LT/RT = Rotate L/R");
        telemetry.addLine("  DPAD = Precise Movement");
        telemetry.addLine("  A/B/X = Speed Presets");
        telemetry.addLine("  START = Toggle Slow Mode");
        telemetry.addLine("  Y = Toggle Field-Centric");
        telemetry.addLine("  BACK = Reset Position");
        telemetry.addLine("========================================");
        telemetry.addLine("Press START to begin");
        telemetry.addLine("========================================");
        telemetry.update();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        boolean slowMode = false;
        boolean fieldCentric = false;

        waitForStart();
        if (isStopRequested()) return;

        lastUpdateTime = System.nanoTime();

        while (opModeIsActive()) {
            // ========== UPDATE GAMEPAD STATES ==========
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);

            // ========== UPDATE ODOMETRY ==========
            updateDriveEncoderOdometry(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, imu);

            // Calculate actual velocities
            long currentTime = System.nanoTime();
            double dt = (currentTime - lastUpdateTime) / 1e9; // Convert to seconds

            double actualVelX = 0.0;
            double actualVelY = 0.0;
            double actualRotRate = 0.0;

            if (dt > 0.001) { // Avoid division by very small numbers
                actualVelX = (robotX - lastRobotX) / dt;
                actualVelY = (robotY - lastRobotY) / dt;
                actualRotRate = (robotHeading - lastRobotHeading) / dt;

                lastRobotX = robotX;
                lastRobotY = robotY;
                lastRobotHeading = robotHeading;
                lastUpdateTime = currentTime;
            }

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

            // ========== BUTTON HANDLERS ==========
            if (startPressed) slowMode = !slowMode;
            if (yPressed) fieldCentric = !fieldCentric;

            if (aPressed) {
                currentSpeedMode = SpeedMode.SLOW;
            } else if (bPressed) {
                currentSpeedMode = SpeedMode.MEDIUM;
            } else if (xPressed) {
                currentSpeedMode = SpeedMode.FAST;
            }

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
                lastRobotX = 0.0;
                lastRobotY = 0.0;
                lastRobotHeading = 0.0;
                holdX = 0.0;
                holdY = 0.0;
                holdHeading = 0.0;
            }

            if (leftBumperPressed) {
                imu.resetYaw();
                robotHeading = 0.0;
                lastRobotHeading = 0.0;
                holdHeading = 0.0;
            }

            // ========== DRIVE CONTROL INPUT ==========
            double y = 0, x = 0, rx = 0;
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";

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
                x = currentGamepad1.left_stick_y;
                y = currentGamepad1.right_stick_x;
                rx = currentGamepad1.right_trigger - currentGamepad1.left_trigger;

                if (currentGamepad1.dpad_up) {
                    x = -0.3; y = 0; rx = 0;
                } else if (currentGamepad1.dpad_down) {
                    x = 0.3; y = 0; rx = 0;
                } else if (currentGamepad1.dpad_left) {
                    x = 0; y = -0.3; rx = 0;
                } else if (currentGamepad1.dpad_right) {
                    x = 0; y = 0.3; rx = 0;
                }

                maxDrivePower = GAMEPAD1_MAX_POWER;
                activeDriver = "DRIVER 1";
            } else if (gamepad2Active) {
                x = currentGamepad2.left_stick_y;
                y = currentGamepad2.right_stick_x;
                rx = currentGamepad2.right_trigger - currentGamepad2.left_trigger;

                if (currentGamepad2.dpad_up) {
                    x = -0.3; y = 0; rx = 0;
                } else if (currentGamepad2.dpad_down) {
                    x = 0.3; y = 0; rx = 0;
                } else if (currentGamepad2.dpad_left) {
                    x = 0; y = -0.3; rx = 0;
                } else if (currentGamepad2.dpad_right) {
                    x = 0; y = 0.3; rx = 0;
                }

                maxDrivePower = GAMEPAD2_MAX_POWER;
                activeDriver = "DRIVER 2";
            }

            x = applyDeadzone(x);
            y = applyDeadzone(y);
            rx = applyDeadzone(rx);

            // ========== APPLY SPEED MODIFIERS ==========
            double speedMultiplier = 1.0;
            switch (currentSpeedMode) {
                case SLOW: speedMultiplier = SPEED_SLOW; break;
                case MEDIUM: speedMultiplier = SPEED_MEDIUM; break;
                case FAST: speedMultiplier = SPEED_FAST; break;
            }

            if (slowMode) speedMultiplier = SLOW_MODE_MULTIPLIER;

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

            // ========== ACTIVE RESISTANCE SYSTEM ==========
            // This is the key: resist motion that opposes driver intent

            boolean isMoving = (Math.abs(x) > 0.01 || Math.abs(y) > 0.01 || Math.abs(rx) > 0.01);
            String resistanceMode = "NONE";
            double resistX = 0.0;
            double resistY = 0.0;
            double resistRx = 0.0;

            if (isMoving) {
                // MOVING MODE: Maintain intended velocity, resist deviations
                // Estimate target velocity based on commanded inputs
                // Rough conversion: assume max stick input = ~30 inches/sec at full power
                final double MAX_LINEAR_VELOCITY = 30.0; // inches/sec at full power
                final double MAX_ROTATION_RATE = 3.0;    // radians/sec at full power

                targetVelocityX = x * MAX_LINEAR_VELOCITY;
                targetVelocityY = y * MAX_LINEAR_VELOCITY;
                targetRotationRate = rx * MAX_ROTATION_RATE;

                // Calculate velocity errors
                double velErrorX = targetVelocityX - actualVelX;
                double velErrorY = targetVelocityY - actualVelY;
                double rotErrorRate = targetRotationRate - actualRotRate;

                // Apply resistance to velocity deviations
                resistX = velErrorX * VELOCITY_RESISTANCE_GAIN * 0.03; // Scale down for power units
                resistY = velErrorY * VELOCITY_RESISTANCE_GAIN * 0.03;
                resistRx = rotErrorRate * VELOCITY_RESISTANCE_GAIN * 0.3;

                resistanceMode = "VELOCITY";

                // Update hold position for when we stop
                holdX = robotX;
                holdY = robotY;
                holdHeading = robotHeading;

            } else {
                // STOPPED MODE: Hold position, resist any movement
                targetVelocityX = 0.0;
                targetVelocityY = 0.0;
                targetRotationRate = 0.0;

                // Position-based resistance
                double errorX = holdX - robotX;
                double errorY = holdY - robotY;
                double errorHeading = holdHeading - robotHeading;

                // Normalize heading error
                while (errorHeading > Math.PI) errorHeading -= 2 * Math.PI;
                while (errorHeading < -Math.PI) errorHeading += 2 * Math.PI;

                double distanceError = Math.sqrt(errorX * errorX + errorY * errorY);

                if (distanceError > POSITION_DEADZONE) {
                    resistX = errorX * POSITION_HOLD_KP;
                    resistY = errorY * POSITION_HOLD_KP;
                    resistanceMode = "POSITION";
                }

                if (Math.abs(Math.toDegrees(errorHeading)) > ROTATION_DEADZONE) {
                    resistRx = errorHeading * ROTATION_HOLD_KP;
                    if (resistanceMode.equals("POSITION")) {
                        resistanceMode = "POSITION+ROTATION";
                    } else {
                        resistanceMode = "ROTATION";
                    }
                }

                // Also resist any unwanted velocity when stopped
                if (Math.abs(actualVelX) > VELOCITY_DEADZONE || Math.abs(actualVelY) > VELOCITY_DEADZONE) {
                    resistX -= actualVelX * VELOCITY_RESISTANCE_GAIN * 0.03;
                    resistY -= actualVelY * VELOCITY_RESISTANCE_GAIN * 0.03;
                }
            }

            // Limit resistance power
            double resistMag = Math.sqrt(resistX * resistX + resistY * resistY);
            if (resistMag > MAX_RESISTANCE_POWER) {
                resistX = (resistX / resistMag) * MAX_RESISTANCE_POWER;
                resistY = (resistY / resistMag) * MAX_RESISTANCE_POWER;
            }
            resistRx = Range.clip(resistRx, -MAX_RESISTANCE_POWER, MAX_RESISTANCE_POWER);

            // Add resistance to driver commands
            x += resistX;
            y += resistY;
            rx += resistRx;

            // ========== MECANUM DRIVE CALCULATION ==========
            double frontLeftPower = -y + x + rx;
            double backLeftPower = -y - x + rx;
            double frontRightPower = -y - x - rx;
            double backRightPower = -y + x - rx;

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

            frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
            backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
            frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
            backRightPower = Range.clip(backRightPower, -1.0, 1.0);

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ========== TELEMETRY ==========
            telemetry.addLine("========== ACTIVE RESISTANCE MODE ==========");
            telemetry.addData("Active Driver", activeDriver);

            String speedModeStr = slowMode ? "SLOW MODE (30%)" :
                    currentSpeedMode == SpeedMode.SLOW ? "SLOW (30%)" :
                            currentSpeedMode == SpeedMode.MEDIUM ? "MEDIUM (60%)" :
                                    "FAST (100%)";
            telemetry.addData("Speed", speedModeStr);
            telemetry.addData("Control Mode", fieldCentric ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Resistance", resistanceMode + " 🛡️");
            telemetry.addLine();

            telemetry.addLine("--- Position & Velocity ---");
            telemetry.addData("X Position", "%.2f in", robotX);
            telemetry.addData("Y Position", "%.2f in", robotY);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(robotHeading));
            telemetry.addData("Velocity X", "%.1f in/s", actualVelX);
            telemetry.addData("Velocity Y", "%.1f in/s", actualVelY);
            telemetry.addLine();

            if (isMoving) {
                telemetry.addLine("--- Velocity Tracking ---");
                telemetry.addData("Target Vel X", "%.1f in/s", targetVelocityX);
                telemetry.addData("Target Vel Y", "%.1f in/s", targetVelocityY);
                telemetry.addData("Vel Error", "%.1f in/s",
                        Math.sqrt(Math.pow(targetVelocityX - actualVelX, 2) +
                                Math.pow(targetVelocityY - actualVelY, 2)));
            } else {
                telemetry.addLine("--- Position Hold ---");
                double posError = Math.sqrt(Math.pow(holdX - robotX, 2) +
                        Math.pow(holdY - robotY, 2));
                double headError = Math.toDegrees(holdHeading - robotHeading);
                telemetry.addData("Position Error", "%.2f in", posError);
                telemetry.addData("Heading Error", "%.1f°", headError);
            }
            telemetry.addLine();

            telemetry.addLine("--- Resistance Powers ---");
            telemetry.addData("Resist X", "%.2f", resistX);
            telemetry.addData("Resist Y", "%.2f", resistY);
            telemetry.addData("Resist Rx", "%.2f", resistRx);
            telemetry.addLine();

            telemetry.addLine("--- Drive Powers ---");
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BR", "%.2f", backRightPower);

            telemetry.update();
        }
    }

    // ========== HELPER METHODS ==========

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

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

@TeleOp(name = "Motion Only Mode")
public class JustMotion extends LinearOpMode {
    
    // ========== MOTOR POWER CONSTANTS ==========
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;
    
    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 1.0;  // Full power for driver 1
    private static final double GAMEPAD2_MAX_POWER = 0.5;  // Half power for driver 2
    
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
    
    // ========== STATE VARIABLES ==========
    private double robotX = 0.0;  // in inches
    private double robotY = 0.0;  // in inches
    private double robotHeading = 0.0;  // in radians
    
    private int lastFLEncoder = 0;
    private int lastBLEncoder = 0;
    private int lastFREncoder = 0;
    private int lastBREncoder = 0;
    private double currentPos;
    
    // Speed preset state
    private enum SpeedMode { SLOW, MEDIUM, FAST }
    private SpeedMode currentSpeedMode = SpeedMode.FAST;

    @Override
    public void runOpMode() throws InterruptedException {
        // ========== HARDWARE INITIALIZATION ==========
        
        // Drive motors

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor launchMotor = hardwareMap.dcMotor.get("launchMotor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Configure encoders for drive motors
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // Initialize servos
        Servo spinSpinServo = hardwareMap.get(Servo.class, "spinSpinServo");
        Servo spatulaServo = hardwareMap.get(Servo.class, "spatulaServo");
        Servo stopServo = hardwareMap.get(Servo.class, "stopServo");

        // ========== INIT TELEMETRY ==========
        telemetry.addLine("========================================");
        telemetry.addLine("DRIVE ONLY MODE");
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
                currentSpeedMode = SpeedMode.SLOW;
            } else if (bPressed) {
                currentSpeedMode = SpeedMode.MEDIUM;
            } else if (xPressed) {
                currentSpeedMode = SpeedMode.FAST;
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
                                 currentSpeedMode == SpeedMode.SLOW ? "SLOW (30%)" :
                                 currentSpeedMode == SpeedMode.MEDIUM ? "MEDIUM (60%)" :
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

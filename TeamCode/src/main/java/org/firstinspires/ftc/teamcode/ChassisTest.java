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

@TeleOp(name = "Chassis Test ONLY")
public class ChassisTestTeleOp extends LinearOpMode {
    
    // ========== MOTOR POWER CONSTANTS ==========
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;
    
    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 0.5;
    private static final double GAMEPAD2_MAX_POWER = 0.5;
    private static final double FULL_POWER = 1.0;  // For testing individual motors
    
    // ========== ODOMETRY CONSTANTS ==========
    private static final double WHEEL_DIAMETER_MM = 32.0;
    private static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;
    private static final double COUNTS_PER_REV = 2000.0;
    private static final double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
    
    // ========== STATE VARIABLES ==========
    private double robotX = 0.0;  // in inches
    private double robotY = 0.0;  // in inches
    private double robotHeading = 0.0;  // in radians
    
    private int lastXOdoCount = 0;
    private int lastYOdoCount = 0;
    
    // Test mode for individual motor control
    private enum TestMode {
        NORMAL_DRIVE,
        TEST_FRONT_LEFT,
        TEST_BACK_LEFT,
        TEST_FRONT_RIGHT,
        TEST_BACK_RIGHT,
        TEST_LEFT_SIDE,
        TEST_RIGHT_SIDE,
        TEST_FRONT_PAIR,
        TEST_BACK_PAIR
    }
    
    private TestMode currentTestMode = TestMode.NORMAL_DRIVE;

    @Override
    public void runOpMode() throws InterruptedException {
        // ========== HARDWARE INITIALIZATION ==========
        
        // Drive motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        
        // Odometry encoders
        DcMotor xOdo = hardwareMap.dcMotor.get("xOdo");
        DcMotor yOdo = hardwareMap.dcMotor.get("yOdo");
        
        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
        
        // Configure odometry encoders
        xOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
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
        
        // ========== INIT TELEMETRY ==========
        telemetry.addLine("========================================");
        telemetry.addLine("CHASSIS TEST MODE");
        telemetry.addLine("========================================");
        telemetry.addData("Drive Motors", "✓");
        telemetry.addData("Odometry Wheels", "✓");
        telemetry.addData("IMU", "✓");
        telemetry.addLine("========================================");
        telemetry.addLine("DPAD Controls:");
        telemetry.addLine("  UP = Normal Drive");
        telemetry.addLine("  DOWN = Test Individual Motors");
        telemetry.addLine("  LEFT = Test Motor Pairs");
        telemetry.addLine("  RIGHT = Test Sides");
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
        boolean slowMode = false;
        
        waitForStart();
        if (isStopRequested()) return;
        
        while (opModeIsActive()) {
            // ========== UPDATE GAMEPAD STATES ==========
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad2.copy(gamepad2);
            
            // ========== UPDATE ODOMETRY ==========
            updateOdometry(xOdo, yOdo, imu);
            
            // ========== RISING EDGE DETECTION ==========
            boolean startPressed = (currentGamepad1.start && !previousGamepad1.start) || 
                                 (currentGamepad2.start && !previousGamepad2.start);
            boolean backPressed = (currentGamepad1.back && !previousGamepad1.back) || 
                                (currentGamepad2.back && !previousGamepad2.back);
            boolean dpadUpPressed = (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || 
                                   (currentGamepad2.dpad_up && !previousGamepad2.dpad_up);
            boolean dpadDownPressed = (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) || 
                                     (currentGamepad2.dpad_down && !previousGamepad2.dpad_down);
            boolean dpadLeftPressed = (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) || 
                                     (currentGamepad2.dpad_left && !previousGamepad2.dpad_left);
            boolean dpadRightPressed = (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) || 
                                      (currentGamepad2.dpad_right && !previousGamepad2.dpad_right);
            boolean aPressed = (currentGamepad1.a && !previousGamepad1.a) || 
                             (currentGamepad2.a && !previousGamepad2.a);
            boolean bPressed = (currentGamepad1.b && !previousGamepad1.b) || 
                             (currentGamepad2.b && !previousGamepad2.b);
            boolean xPressed = (currentGamepad1.x && !previousGamepad1.x) || 
                             (currentGamepad2.x && !previousGamepad2.x);
            boolean yPressed = (currentGamepad1.y && !previousGamepad1.y) || 
                             (currentGamepad2.y && !previousGamepad2.y);
            
            // ========== TEST MODE SELECTION ==========
            if (dpadUpPressed) {
                currentTestMode = TestMode.NORMAL_DRIVE;
            } else if (dpadDownPressed) {
                // Cycle through individual motor tests
                switch (currentTestMode) {
                    case NORMAL_DRIVE:
                    case TEST_BACK_PAIR:
                    case TEST_RIGHT_SIDE:
                    case TEST_FRONT_PAIR:
                    case TEST_LEFT_SIDE:
                        currentTestMode = TestMode.TEST_FRONT_LEFT;
                        break;
                    case TEST_FRONT_LEFT:
                        currentTestMode = TestMode.TEST_BACK_LEFT;
                        break;
                    case TEST_BACK_LEFT:
                        currentTestMode = TestMode.TEST_FRONT_RIGHT;
                        break;
                    case TEST_FRONT_RIGHT:
                        currentTestMode = TestMode.TEST_BACK_RIGHT;
                        break;
                    case TEST_BACK_RIGHT:
                        currentTestMode = TestMode.TEST_FRONT_LEFT;
                        break;
                }
            } else if (dpadLeftPressed) {
                // Cycle through motor pairs
                if (currentTestMode == TestMode.TEST_FRONT_PAIR) {
                    currentTestMode = TestMode.TEST_BACK_PAIR;
                } else {
                    currentTestMode = TestMode.TEST_FRONT_PAIR;
                }
            } else if (dpadRightPressed) {
                // Cycle through sides
                if (currentTestMode == TestMode.TEST_LEFT_SIDE) {
                    currentTestMode = TestMode.TEST_RIGHT_SIDE;
                } else {
                    currentTestMode = TestMode.TEST_LEFT_SIDE;
                }
            }
            
            // ========== START BUTTON - SLOW MODE TOGGLE ==========
            if (startPressed) {
                slowMode = !slowMode;
            }
            
            // ========== BACK BUTTON - RESET IMU AND ODOMETRY ==========
            if (backPressed) {
                imu.resetYaw();
                xOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                yOdo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                xOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                yOdo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robotX = 0.0;
                robotY = 0.0;
                robotHeading = 0.0;
                lastXOdoCount = 0;
                lastYOdoCount = 0;
            }
            
            // ========== A BUTTON - EMERGENCY STOP ==========
            if (aPressed) {
                frontLeftMotor.setPower(0);
                backLeftMotor.setPower(0);
                frontRightMotor.setPower(0);
                backRightMotor.setPower(0);
            }
            
            // ========== MOTOR CONTROL ==========
            double frontLeftPower = 0;
            double backLeftPower = 0;
            double frontRightPower = 0;
            double backRightPower = 0;
            
            // Check for active gamepad
            boolean gamepad1Active = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE;
            
            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE;
            
            double x = 0, y = 0, rx = 0;
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";
            
            if (currentTestMode == TestMode.NORMAL_DRIVE) {
                // ========== NORMAL MECANUM DRIVE ==========
                if (gamepad1Active) {
                    x = currentGamepad1.left_stick_y;
                    y = currentGamepad1.right_stick_x;
                    rx = currentGamepad1.right_trigger - currentGamepad1.left_trigger;
                    maxDrivePower = GAMEPAD1_MAX_POWER;
                    activeDriver = "DRIVER 1";
                } else if (gamepad2Active) {
                    x = currentGamepad2.left_stick_y;
                    y = currentGamepad2.right_stick_x;
                    rx = currentGamepad2.right_trigger - currentGamepad2.left_trigger;
                    maxDrivePower = GAMEPAD2_MAX_POWER;
                    activeDriver = "DRIVER 2";
                }
                
                // Apply deadzone
                x = applyDeadzone(x);
                y = applyDeadzone(y);
                rx = applyDeadzone(rx);
                
                // Apply slow mode
                if (slowMode) {
                    x *= SLOW_MODE_MULTIPLIER;
                    y *= SLOW_MODE_MULTIPLIER;
                    rx *= SLOW_MODE_MULTIPLIER;
                }
                
                // Mecanum drive calculation
                frontLeftPower = -y + x + rx;
                backLeftPower = -y - x + rx;
                frontRightPower = -y - x - rx;
                backRightPower = -y + x - rx;
                
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
                
                frontLeftPower = Range.clip(frontLeftPower, -0.5, 0.5);
                backLeftPower = Range.clip(backLeftPower, -0.5, 0.5);
                frontRightPower = Range.clip(frontRightPower, -0.5, 0.5);
                backRightPower = Range.clip(backRightPower, -0.5, 0.5);
                
            } else {
                // ========== INDIVIDUAL MOTOR TEST MODE ==========
                // Use left stick Y for forward/backward control
                double testPower = gamepad1Active ? currentGamepad1.left_stick_y : 
                                  gamepad2Active ? currentGamepad2.left_stick_y : 0;
                testPower = applyDeadzone(testPower);
                
                switch (currentTestMode) {
                    case TEST_FRONT_LEFT:
                        frontLeftPower = testPower;
                        break;
                    case TEST_BACK_LEFT:
                        backLeftPower = testPower;
                        break;
                    case TEST_FRONT_RIGHT:
                        frontRightPower = testPower;
                        break;
                    case TEST_BACK_RIGHT:
                        backRightPower = testPower;
                        break;
                    case TEST_LEFT_SIDE:
                        frontLeftPower = testPower;
                        backLeftPower = testPower;
                        break;
                    case TEST_RIGHT_SIDE:
                        frontRightPower = testPower;
                        backRightPower = testPower;
                        break;
                    case TEST_FRONT_PAIR:
                        frontLeftPower = testPower;
                        frontRightPower = testPower;
                        break;
                    case TEST_BACK_PAIR:
                        backLeftPower = testPower;
                        backRightPower = testPower;
                        break;
                }
            }
            
            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            // ========== TELEMETRY ==========
            telemetry.addLine("========== CHASSIS TEST MODE ==========");
            telemetry.addData("Test Mode", currentTestMode);
            telemetry.addLine();
            
            telemetry.addLine("--- Mode Selection ---");
            telemetry.addLine("DPAD UP: Normal Drive");
            telemetry.addLine("DPAD DOWN: Cycle Individual Motors");
            telemetry.addLine("DPAD LEFT: Cycle Motor Pairs");
            telemetry.addLine("DPAD RIGHT: Cycle Sides");
            telemetry.addLine();
            
            if (currentTestMode == TestMode.NORMAL_DRIVE) {
                telemetry.addData("Active Driver", activeDriver);
                telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
            } else {
                telemetry.addLine("Use LEFT STICK Y to control test motor(s)");
            }
            telemetry.addLine();
            
            telemetry.addLine("--- Position (Odometry) ---");
            telemetry.addData("X Position", "%.2f inches", robotX);
            telemetry.addData("Y Position", "%.2f inches", robotY);
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(robotHeading));
            telemetry.addLine();
            
            telemetry.addLine("--- Encoder Counts ---");
            telemetry.addData("X Odo", xOdo.getCurrentPosition());
            telemetry.addData("Y Odo", yOdo.getCurrentPosition());
            telemetry.addData("FL Encoder", frontLeftMotor.getCurrentPosition());
            telemetry.addData("BL Encoder", backLeftMotor.getCurrentPosition());
            telemetry.addData("FR Encoder", frontRightMotor.getCurrentPosition());
            telemetry.addData("BR Encoder", backRightMotor.getCurrentPosition());
            telemetry.addLine();
            
            telemetry.addLine("--- Motor Powers ---");
            telemetry.addData("FL", "%.2f %s", frontLeftPower, currentTestMode == TestMode.TEST_FRONT_LEFT ? "◄◄◄" : "");
            telemetry.addData("BL", "%.2f %s", backLeftPower, currentTestMode == TestMode.TEST_BACK_LEFT ? "◄◄◄" : "");
            telemetry.addData("FR", "%.2f %s", frontRightPower, currentTestMode == TestMode.TEST_FRONT_RIGHT ? "◄◄◄" : "");
            telemetry.addData("BR", "%.2f %s", backRightPower, currentTestMode == TestMode.TEST_BACK_RIGHT ? "◄◄◄" : "");
            
            telemetry.update();
        }
    }
    
    // ========== HELPER METHODS ==========
    
    private void updateOdometry(DcMotor xOdo, DcMotor yOdo, IMU imu) {
        // Get current encoder counts
        int currentXCount = xOdo.getCurrentPosition();
        int currentYCount = yOdo.getCurrentPosition();
        
        // Calculate change in counts
        int deltaX = currentXCount - lastXOdoCount;
        int deltaY = currentYCount - lastYOdoCount;
        
        // Convert to inches
        double deltaXInches = deltaX / COUNTS_PER_INCH;
        double deltaYInches = deltaY / COUNTS_PER_INCH;
        
        // Update heading from IMU
        robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        
        // Update global position (field-centric)
        robotX += deltaXInches * Math.cos(robotHeading) - deltaYInches * Math.sin(robotHeading);
        robotY += deltaXInches * Math.sin(robotHeading) + deltaYInches * Math.cos(robotHeading);
        
        // Store current counts for next iteration
        lastXOdoCount = currentXCount;
        lastYOdoCount = currentYCount;
    }
    
    private double applyDeadzone(double value) {
        return Math.abs(value) < JOYSTICK_DEADZONE ? 0 : value;
    }
}

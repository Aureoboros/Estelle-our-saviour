package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "New Robot Teleop v2")
public class JustMotion extends LinearOpMode {
    
    // ========== MOTOR POWER CONSTANTS ==========
    private static final double INTAKE_POWER = 1.0;
    private static final double LAUNCH_MOTOR_MIN = 0.5;
    private static final double LAUNCH_MOTOR_MAX = 1.0;
    private static final double LAUNCH_MOTOR_STEP = 0.05;
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;
    
    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 0.5;
    private static final double GAMEPAD2_MAX_POWER = 0.5;
    
    // ========== ODOMETRY CONSTANTS ==========
    private static final double WHEEL_DIAMETER_MM = 32.0;
    private static final double WHEEL_DIAMETER_INCHES = WHEEL_DIAMETER_MM / 25.4;
    private static final double COUNTS_PER_REV = 2000.0;
    private static final double COUNTS_PER_INCH = COUNTS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
    
    // ========== SERVO CONSTANTS ==========
    private static final double SPINDEXER_POSITION_1 = 0.0;
    private static final double SPINDEXER_POSITION_2 = 0.33;  // 120° rotation
    private static final double SPINDEXER_POSITION_3 = 0.67;  // 240° rotation
    private static final int SPINDEXER_DELAY_MS = 300;  // Delay between spindexer rotations
    
    private static final double TURRET_SEARCH_SPEED = 0.3;
    private static final double TURRET_AIM_SPEED = 0.15;
    private static final double TURRET_STOP = 0.5;  // Continuous servo stopped position
    private static final double ANGLE_TOLERANCE_DEGREES = 3.0;
    
    // ========== APRILTAG CONFIGURATION ==========
    private static final int RED_APRILTAG_ID = 24;
    private static final int BLUE_APRILTAG_ID = 20;
    
    // ========== ALLIANCE SELECTION ==========
    private enum Alliance { NONE, RED, BLUE }
    private Alliance selectedAlliance = Alliance.NONE;
    
    // ========== STATE VARIABLES ==========
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    
    // Odometry tracking
    private double robotX = 0.0;  // in inches
    private double robotY = 0.0;  // in inches
    private double robotHeading = 0.0;  // in radians
    
    private int lastXOdoCount = 0;
    private int lastYOdoCount = 0;
    
    // Spindexer state
    private int spindexerPosition = 0;  // 0, 1, or 2
    
    // Turret state
    private boolean turretSearching = false;
    private double turretSearchStartTime = 0.0;
    private static final double TURRET_SEARCH_TIMEOUT = 3.0;  // seconds for full 360° search

    @Override
    public void runOpMode() throws InterruptedException {
        // ========== HARDWARE INITIALIZATION ==========
        
        // Drive motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        
        // Mechanism motors
        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor launchMotor = hardwareMap.dcMotor.get("launchMotor");
        
        // Odometry encoders
        DcMotor xOdo = hardwareMap.dcMotor.get("xOdo");
        DcMotor yOdo = hardwareMap.dcMotor.get("yOdo");
        
        // Servos
        Servo spindexerServo = hardwareMap.servo.get("spindexerServo");
        CRServo turretServo = hardwareMap.crservo.get("turretServo");
        
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
        
        // Set motor modes for mechanisms
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
        
        // Initialize AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        
        // Initialize vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        
        // Initialize servos
        spindexerServo.setPosition(SPINDEXER_POSITION_1);
        turretServo.setPower(TURRET_STOP);
        
        // ========== INIT TELEMETRY ==========
        telemetry.addLine("========================================");
        telemetry.addLine("NEW ROBOT INITIALIZED");
        telemetry.addLine("========================================");
        telemetry.addData("Drive Motors", "✓");
        telemetry.addData("Intake Motor", "✓");
        telemetry.addData("Launch Motor", "✓");
        telemetry.addData("Spindexer Servo", "✓");
        telemetry.addData("Turret Servo", "✓");
        telemetry.addData("Odometry Wheels", "✓");
        telemetry.addData("IMU", "✓");
        telemetry.addData("Camera", visionPortal.getCameraState());
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
        boolean intakeActive = false;
        boolean launchActive = false;
        boolean turretAiming = false;
        
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
            
            // ========== RISING EDGE DETECTION HELPER ==========
            boolean bPressed = (currentGamepad1.b && !previousGamepad1.b) || 
                             (currentGamepad2.b && !previousGamepad2.b);
            boolean xPressed = (currentGamepad1.x && !previousGamepad1.x) || 
                             (currentGamepad2.x && !previousGamepad2.x);
            boolean yPressed = (currentGamepad1.y && !previousGamepad1.y) || 
                             (currentGamepad2.y && !previousGamepad2.y);
            boolean aPressed = (currentGamepad1.a && !previousGamepad1.a) || 
                             (currentGamepad2.a && !previousGamepad2.a);
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
            
            // ========== DPAD UP/DOWN - ALLIANCE SELECTION ==========
            if (dpadUpPressed) {
                selectedAlliance = Alliance.BLUE;
            } else if (dpadDownPressed) {
                selectedAlliance = Alliance.RED;
            }
            
            // ========== DPAD LEFT/RIGHT - LAUNCH MOTOR SPEED ADJUSTMENT ==========
            if (dpadLeftPressed) {
                // Decrease launch power
                launchMotorPower -= LAUNCH_MOTOR_STEP;
                if (launchMotorPower < LAUNCH_MOTOR_MIN) {
                    launchMotorPower = LAUNCH_MOTOR_MAX;  // Wrap to 100%
                }
                // Update launcher if active
                if (launchActive) {
                    launchMotor.setPower(launchMotorPower);
                }
            } else if (dpadRightPressed) {
                // Increase launch power
                launchMotorPower += LAUNCH_MOTOR_STEP;
                if (launchMotorPower > LAUNCH_MOTOR_MAX) {
                    launchMotorPower = LAUNCH_MOTOR_MIN;  // Wrap to 50%
                }
                // Update launcher if active
                if (launchActive) {
                    launchMotor.setPower(launchMotorPower);
                }
            }
            
            // ========== B BUTTON - INTAKE TOGGLE WITH SPINDEXER ==========
            if (bPressed) {
                intakeActive = !intakeActive;
                
                if (intakeActive) {
                    // Start intake sequence
                    intakeMotor.setPower(INTAKE_POWER);
                    
                    // Rotate spindexer through 3 positions (0 -> 1 -> 2)
                    new Thread(() -> {
                        try {
                            for (int i = 0; i < 2; i++) {
                                spindexerPosition = (spindexerPosition + 1) % 3;
                                double targetPos = spindexerPosition == 0 ? SPINDEXER_POSITION_1 :
                                                 spindexerPosition == 1 ? SPINDEXER_POSITION_2 :
                                                 SPINDEXER_POSITION_3;
                                spindexerServo.setPosition(targetPos);
                                Thread.sleep(SPINDEXER_DELAY_MS);
                            }
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }).start();
                } else {
                    // Stop intake
                    intakeMotor.setPower(0);
                }
            }
            
            // ========== X BUTTON - AIM AND SHOOT ==========
            if (xPressed) {
                if (!launchActive) {
                    // Start launch sequence
                    launchActive = true;
                    turretAiming = true;
                    turretSearching = true;
                    turretSearchStartTime = getRuntime();
                    
                    // Spin up launcher
                    launchMotor.setPower(launchMotorPower);
                } else {
                    // Stop launch
                    launchActive = false;
                    turretAiming = false;
                    turretSearching = false;
                    launchMotor.setPower(0);
                    turretServo.setPower(TURRET_STOP);
                }
            }
            
            // ========== Y BUTTON - REVERSE INTAKE ==========
            if (yPressed) {
                if (intakeActive) {
                    // Reverse intake
                    intakeMotor.setPower(-INTAKE_POWER);
                } else {
                    intakeMotor.setPower(0);
                }
            }
            
            // ========== A BUTTON - EMERGENCY STOP ==========
            if (aPressed) {
                intakeActive = false;
                launchActive = false;
                turretAiming = false;
                turretSearching = false;
                
                intakeMotor.setPower(0);
                launchMotor.setPower(0);
                turretServo.setPower(TURRET_STOP);
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
            
            // ========== TURRET AUTO-AIM CONTROL ==========
            if (turretAiming) {
                // Determine target AprilTag based on alliance
                int targetTagId = (selectedAlliance == Alliance.RED) ? RED_APRILTAG_ID : 
                                 (selectedAlliance == Alliance.BLUE) ? BLUE_APRILTAG_ID : 
                                 BLUE_APRILTAG_ID;  // Default to blue if none selected
                
                AprilTagDetection target = getAprilTagDetection(targetTagId);
                
                if (target != null) {
                    // Tag found! Aim at it
                    turretSearching = false;
                    double yawError = target.ftcPose.yaw;
                    
                    if (Math.abs(yawError) < ANGLE_TOLERANCE_DEGREES) {
                        // On target
                        turretServo.setPower(TURRET_STOP);
                    } else {
                        // Proportional aiming
                        double aimPower = Math.signum(yawError) * TURRET_AIM_SPEED;
                        turretServo.setPower(TURRET_STOP + aimPower);
                    }
                } else if (turretSearching) {
                    // Search for tag
                    double searchTime = getRuntime() - turretSearchStartTime;
                    
                    if (searchTime < TURRET_SEARCH_TIMEOUT) {
                        // Continue searching (rotate)
                        turretServo.setPower(TURRET_STOP + TURRET_SEARCH_SPEED);
                    } else {
                        // Timeout - return to center
                        turretSearching = false;
                        turretServo.setPower(TURRET_STOP);
                    }
                }
            } else {
                // Not aiming, keep turret stopped
                turretServo.setPower(TURRET_STOP);
            }
            
            // ========== MANUAL DRIVE CONTROL ==========
            double y = 0, x = 0, rx = 0;
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";
            
            // Check for active gamepad
            boolean gamepad1Active = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE;
            
            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE;
            
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
            
            frontLeftPower = Range.clip(frontLeftPower, -0.5, 0.5);
            backLeftPower = Range.clip(backLeftPower, -0.5, 0.5);
            frontRightPower = Range.clip(frontRightPower, -0.5, 0.5);
            backRightPower = Range.clip(backRightPower, -0.5, 0.5);
            
            // Set motor powers
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            // ========== TELEMETRY ==========
            telemetry.addLine("========== ROBOT STATUS ==========");
            telemetry.addData("Alliance", selectedAlliance);
            telemetry.addData("Target AprilTag", selectedAlliance == Alliance.RED ? RED_APRILTAG_ID : 
                                                selectedAlliance == Alliance.BLUE ? BLUE_APRILTAG_ID : 
                                                "None Selected");
            telemetry.addData("Active Driver", activeDriver);
            telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
            telemetry.addLine();
            
            telemetry.addLine("--- Position (Odometry) ---");
            telemetry.addData("X Position", "%.2f inches", robotX);
            telemetry.addData("Y Position", "%.2f inches", robotY);
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(robotHeading));
            telemetry.addLine();
            
            telemetry.addLine("--- Mechanisms ---");
            telemetry.addData("Intake", intakeActive ? "RUNNING" : "STOPPED");
            telemetry.addData("Launch Power", "%.0f%% (DPAD L/R to adjust)", launchMotorPower * 100);
            telemetry.addData("Launch Status", launchActive ? "ACTIVE" : "STOPPED");
            telemetry.addData("Spindexer Pos", spindexerPosition);
            
            if (turretAiming) {
                AprilTagDetection target = getAprilTagDetection(TARGET_APRILTAG_ID);
                if (target != null) {
                    telemetry.addData("Turret", "LOCKED (%.1f°)", target.ftcPose.yaw);
                } else if (turretSearching) {
                    telemetry.addData("Turret", "SEARCHING...");
                } else {
                    telemetry.addData("Turret", "TARGET LOST");
                }
            } else {
                telemetry.addData("Turret", "IDLE");
            }
            
            telemetry.addLine();
            telemetry.addLine("--- Drive Motors ---");
            telemetry.addData("FL", "%.2f", frontLeftPower);
            telemetry.addData("BL", "%.2f", backLeftPower);
            telemetry.addData("FR", "%.2f", frontRightPower);
            telemetry.addData("BR", "%.2f", backRightPower);
            
            telemetry.update();
        }
        
        // Clean up
        visionPortal.close();
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
    
    private AprilTagDetection getAprilTagDetection(int targetId) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == targetId) {
                return detection;
            }
        }
        return null;
    }
    
    private double applyDeadzone(double value) {
        return Math.abs(value) < JOYSTICK_DEADZONE ? 0 : value;
    }
}

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

// Limelight imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@TeleOp(name = "Phase 2 - Limelight Fusion")
public class Phase2LimelightTeleop extends LinearOpMode {
    
    // ========== MOTOR POWER CONSTANTS ==========
    private static final double INTAKE_POWER = 1.0;
    private static final double LAUNCH_MOTOR_MIN = 0.5;
    private static final double LAUNCH_MOTOR_MAX = 1.0;
    private static final double LAUNCH_MOTOR_STEP = 0.05;
    private static final double SLOW_MODE_MULTIPLIER = 0.3;
    private static final double JOYSTICK_DEADZONE = 0.1;
    
    // Armor motor constants
    private static final double ARMOR_DEPLOY_POWER = 0.4;
    private static final double ARMOR_RETRACT_POWER = -0.8;
    private static final int ARMOR_RETRACT_TIME_MS = 500;
    
    // Driver-specific power limits
    private static final double GAMEPAD1_MAX_POWER = 0.5;
    private static final double GAMEPAD2_MAX_POWER = 0.5;
    
    // ========== DRIVE ENCODER ODOMETRY CONSTANTS ==========
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                  (WHEEL_DIAMETER_INCHES * Math.PI);
    
    // ========== SERVO CONSTANTS ==========
    private static final double SPINDEXER_POSITION_1 = 0.0;
    private static final double SPINDEXER_POSITION_2 = 0.33;
    private static final double SPINDEXER_POSITION_3 = 0.67;
    private static final int SPINDEXER_DELAY_MS = 300;
    
    private static final double TURRET_SEARCH_SPEED = 0.3;
    private static final double TURRET_AIM_SPEED = 0.15;
    private static final double TURRET_STOP = 0.5;
    private static final double ANGLE_TOLERANCE_DEGREES = 3.0;
    
    // ========== APRILTAG CONFIGURATION ==========
    private static final int RED_APRILTAG_ID = 24;
    private static final int BLUE_APRILTAG_ID = 20;
    
    // ========== POSITION FUSION CONSTANTS ==========
    private static final double VISION_CORRECTION_THRESHOLD_LARGE = 6.0;  // inches
    private static final double VISION_CORRECTION_THRESHOLD_SMALL = 2.0;  // inches
    private static final double VISION_BLEND_RATIO = 0.3;  // 30% vision, 70% odometry for small corrections
    private static final double POSITION_CONFIDENCE_DECAY = 0.02;  // Confidence drops 2% per loop without vision
    private static final double PUSH_DETECTION_THRESHOLD = 3.0;  // inches of unexpected movement
    private static final double AUTO_NAV_SPEED = 0.25;
    private static final double AUTO_NAV_POSITION_TOLERANCE = 4.0;  // inches
    
    // ========== ALLIANCE SELECTION ==========
    private enum Alliance { NONE, RED, BLUE }
    private Alliance selectedAlliance = Alliance.NONE;
    
    // ========== POSITION SOURCE TRACKING ==========
    private enum PositionSource { ODOMETRY, LIMELIGHT, APRILTAG, FUSED }
    private PositionSource activeSource = PositionSource.ODOMETRY;
    
    // ========== STATE VARIABLES ==========
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Limelight3A limelight;
    
    // Position tracking with fusion
    private double robotX = 0.0;  // Fused position (inches)
    private double robotY = 0.0;
    private double robotHeading = 0.0;  // radians
    
    // Odometry position (may drift)
    private double odometryX = 0.0;
    private double odometryY = 0.0;
    
    // Last vision position (for push detection)
    private double lastVisionX = 0.0;
    private double lastVisionY = 0.0;
    private double lastVisionTime = 0.0;
    
    // Confidence tracking
    private double positionConfidence = 1.0;  // 0.0 to 1.0
    
    // Encoder tracking
    private int lastFLEncoder = 0;
    private int lastBLEncoder = 0;
    private int lastFREncoder = 0;
    private int lastBREncoder = 0;
    
    // Spindexer state
    private int spindexerPosition = 0;
    
    // Turret state
    private boolean turretSearching = false;
    private double turretSearchStartTime = 0.0;
    private static final double TURRET_SEARCH_TIMEOUT = 3.0;
    
    // Auto-navigation state
    private boolean autoNavigating = false;
    private double targetX = 0.0;
    private double targetY = 0.0;

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
        DcMotor leftArmorMotor = hardwareMap.dcMotor.get("leftArmorMotor");
        DcMotor rightArmorMotor = hardwareMap.dcMotor.get("rightArmorMotor");
        
        // Servos
        Servo spindexerServo = hardwareMap.servo.get("spindexerServo");
        CRServo turretServo = hardwareMap.crservo.get("turretServo");
        
        // Set motor directions
        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftArmorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightArmorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        
        // Configure encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set motor modes
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArmorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArmorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        // Set brake behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightArmorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Initialize IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        imu.resetYaw();
        
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);  // Switch to pipeline 0 (configure in Limelight web interface)
        limelight.start();
        
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
        telemetry.addLine("PHASE 2 - LIMELIGHT FUSION");
        telemetry.addLine("========================================");
        telemetry.addData("Drive Motors", "✓");
        telemetry.addData("Mechanisms", "✓");
        telemetry.addData("Armor", "✓ 🛡️");
        telemetry.addData("Limelight", limelight.isConnected() ? "✓ CONNECTED" : "✗ NOT FOUND");
        telemetry.addData("AprilTag Camera", visionPortal.getCameraState());
        telemetry.addData("IMU", "✓");
        telemetry.addLine("========================================");
        telemetry.addLine("Position Fusion: ENABLED");
        telemetry.addLine("  • Odometry (continuous)");
        telemetry.addLine("  • Limelight (correction)");
        telemetry.addLine("  • AprilTag (verification)");
        telemetry.addLine("========================================");
        telemetry.addLine("Press START to begin");
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
        boolean armorDeployed = false;
        double launchMotorPower = 0.75;
        
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
            
            // ========== UPDATE POSITION FUSION ==========
            updatePositionFusion(imu);
            
            // ========== RISING EDGE DETECTION ==========
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
            boolean leftBumperPressed = (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) || 
                                       (currentGamepad2.left_bumper && !previousGamepad2.left_bumper);
            boolean rightBumperPressed = (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) || 
                                        (currentGamepad2.right_bumper && !previousGamepad2.right_bumper);
            
            // ========== DPAD UP/DOWN - ALLIANCE SELECTION ==========
            if (dpadUpPressed) {
                selectedAlliance = Alliance.BLUE;
            } else if (dpadDownPressed) {
                selectedAlliance = Alliance.RED;
            }
            
            // ========== DPAD LEFT/RIGHT - LAUNCH MOTOR SPEED ADJUSTMENT ==========
            if (dpadLeftPressed) {
                launchMotorPower -= LAUNCH_MOTOR_STEP;
                if (launchMotorPower < LAUNCH_MOTOR_MIN) {
                    launchMotorPower = LAUNCH_MOTOR_MAX;
                }
                if (launchActive) {
                    launchMotor.setPower(launchMotorPower);
                }
            } else if (dpadRightPressed) {
                launchMotorPower += LAUNCH_MOTOR_STEP;
                if (launchMotorPower > LAUNCH_MOTOR_MAX) {
                    launchMotorPower = LAUNCH_MOTOR_MIN;
                }
                if (launchActive) {
                    launchMotor.setPower(launchMotorPower);
                }
            }
            
            // ========== RIGHT BUMPER - AUTO-NAVIGATE TO SCORING POSITION ==========
            if (rightBumperPressed) {
                if (!autoNavigating) {
                    // Get target position from alliance-specific AprilTag
                    int targetTagId = (selectedAlliance == Alliance.RED) ? RED_APRILTAG_ID : BLUE_APRILTAG_ID;
                    
                    // Try Limelight first for target position
                    LLResult result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {
                        // Set target to tag position (adjust for scoring offset)
                        targetX = result.getTx();  // Adjust based on your field setup
                        targetY = result.getTy() - 24.0;  // 24 inches in front of tag
                        autoNavigating = true;
                        telemetry.addLine("✓ Auto-nav started (Limelight)");
                    } else {
                        // Fallback to AprilTag
                        AprilTagDetection tag = getAprilTagDetection(targetTagId);
                        if (tag != null) {
                            targetX = calculateTargetXFromTag(tag);
                            targetY = calculateTargetYFromTag(tag);
                            autoNavigating = true;
                            telemetry.addLine("✓ Auto-nav started (AprilTag)");
                        } else {
                            telemetry.addLine("⚠️ No target visible");
                        }
                    }
                } else {
                    // Cancel auto-navigation
                    autoNavigating = false;
                    telemetry.addLine("Auto-nav cancelled");
                }
            }
            
            // ========== LEFT BUMPER - ARMOR TOGGLE ==========
            if (leftBumperPressed) {
                armorDeployed = !armorDeployed;
                
                if (armorDeployed) {
                    leftArmorMotor.setPower(ARMOR_DEPLOY_POWER);
                    rightArmorMotor.setPower(ARMOR_DEPLOY_POWER);
                } else {
                    leftArmorMotor.setPower(ARMOR_RETRACT_POWER);
                    rightArmorMotor.setPower(ARMOR_RETRACT_POWER);
                    
                    new Thread(() -> {
                        try {
                            Thread.sleep(ARMOR_RETRACT_TIME_MS);
                            leftArmorMotor.setPower(0);
                            rightArmorMotor.setPower(0);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }).start();
                }
            }
            
            // ========== B BUTTON - INTAKE TOGGLE ==========
            if (bPressed) {
                intakeActive = !intakeActive;
                
                if (intakeActive) {
                    intakeMotor.setPower(INTAKE_POWER);
                    
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
                    intakeMotor.setPower(0);
                }
            }
            
            // ========== X BUTTON - AIM AND SHOOT ==========
            if (xPressed) {
                if (!launchActive) {
                    launchActive = true;
                    turretAiming = true;
                    turretSearching = true;
                    turretSearchStartTime = getRuntime();
                    launchMotor.setPower(launchMotorPower);
                } else {
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
                armorDeployed = false;
                autoNavigating = false;
                
                intakeMotor.setPower(0);
                launchMotor.setPower(0);
                turretServo.setPower(TURRET_STOP);
                leftArmorMotor.setPower(0);
                rightArmorMotor.setPower(0);
            }
            
            // ========== START BUTTON - SLOW MODE TOGGLE ==========
            if (startPressed) {
                slowMode = !slowMode;
            }
            
            // ========== BACK BUTTON - RESET ALL POSITIONING ==========
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
                odometryX = 0.0;
                odometryY = 0.0;
                positionConfidence = 1.0;
                lastFLEncoder = 0;
                lastBLEncoder = 0;
                lastFREncoder = 0;
                lastBREncoder = 0;
            }
            
            // ========== TURRET AUTO-AIM CONTROL ==========
            if (turretAiming) {
                int targetTagId = (selectedAlliance == Alliance.RED) ? RED_APRILTAG_ID : 
                                 (selectedAlliance == Alliance.BLUE) ? BLUE_APRILTAG_ID : 
                                 BLUE_APRILTAG_ID;
                
                AprilTagDetection target = getAprilTagDetection(targetTagId);
                
                if (target != null) {
                    turretSearching = false;
                    double yawError = target.ftcPose.yaw;
                    
                    if (Math.abs(yawError) < ANGLE_TOLERANCE_DEGREES) {
                        turretServo.setPower(TURRET_STOP);
                    } else {
                        double aimPower = Math.signum(yawError) * TURRET_AIM_SPEED;
                        turretServo.setPower(TURRET_STOP + aimPower);
                    }
                } else if (turretSearching) {
                    double searchTime = getRuntime() - turretSearchStartTime;
                    
                    if (searchTime < TURRET_SEARCH_TIMEOUT) {
                        turretServo.setPower(TURRET_STOP + TURRET_SEARCH_SPEED);
                    } else {
                        turretSearching = false;
                        turretServo.setPower(TURRET_STOP);
                    }
                }
            } else {
                turretServo.setPower(TURRET_STOP);
            }
            
            // ========== DRIVE CONTROL ==========
            double y = 0, x = 0, rx = 0;
            double maxDrivePower = GAMEPAD1_MAX_POWER;
            String activeDriver = "NONE";
            
            // Check for driver override
            boolean gamepad1Active = Math.abs(currentGamepad1.left_stick_y) > JOYSTICK_DEADZONE ||
                                    Math.abs(currentGamepad1.right_stick_x) > JOYSTICK_DEADZONE ||
                                    currentGamepad1.left_trigger > JOYSTICK_DEADZONE ||
                                    currentGamepad1.right_trigger > JOYSTICK_DEADZONE;
            
            boolean gamepad2Active = Math.abs(currentGamepad2.left_stick_y) > JOYSTICK_DEADZONE ||
                                    Math.abs(currentGamepad2.right_stick_x) > JOYSTICK_DEADZONE ||
                                    currentGamepad2.left_trigger > JOYSTICK_DEADZONE ||
                                    currentGamepad2.right_trigger > JOYSTICK_DEADZONE;
            
            if (gamepad1Active || gamepad2Active) {
                autoNavigating = false;  // Driver override cancels auto-nav
            }
            
            if (autoNavigating) {
                // ========== AUTO-NAVIGATION ==========
                double deltaX = targetX - robotX;
                double deltaY = targetY - robotY;
                double distanceToTarget = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                
                if (distanceToTarget < AUTO_NAV_POSITION_TOLERANCE) {
                    // Reached target
                    x = 0;
                    y = 0;
                    rx = 0;
                    autoNavigating = false;
                    telemetry.addLine("✓ Target reached!");
                } else {
                    // Calculate approach vector
                    double angleToTarget = Math.atan2(deltaX, deltaY);
                    double robotRelativeAngle = angleToTarget - robotHeading;
                    
                    // Normalize angle
                    while (robotRelativeAngle > Math.PI) robotRelativeAngle -= 2 * Math.PI;
                    while (robotRelativeAngle < -Math.PI) robotRelativeAngle += 2 * Math.PI;
                    
                    // Calculate movement
                    double speed = Math.min(AUTO_NAV_SPEED, distanceToTarget / 24.0);  // Slow down near target
                    x = Math.sin(robotRelativeAngle) * speed;
                    y = Math.cos(robotRelativeAngle) * speed;
                    
                    // Heading correction
                    double headingError = angleToTarget - robotHeading;
                    while (headingError > Math.PI) headingError -= 2 * Math.PI;
                    while (headingError < -Math.PI) headingError += 2 * Math.PI;
                    rx = headingError * 0.15;
                }
            } else if (gamepad1Active) {
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
            double front

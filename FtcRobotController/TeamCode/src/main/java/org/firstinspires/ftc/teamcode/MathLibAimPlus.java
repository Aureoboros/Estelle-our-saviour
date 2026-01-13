package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Auto-Aiming with Bump Resistance")
public class MathLibAimPlus extends LinearOpMode {
    
    // Motors
    private DcMotor frontRightMotor, backRightMotor, frontLeftMotor, backLeftMotor;
    private DcMotor launchMotor, intakeMotor;
    private DcMotor xodo, yodo;
    
    // Servos
    private Servo spinSpinServo, spatulaServo, stopServo;
    
    // Limelight
    private Limelight3A limelight;
    
    // Constants
    private static final double LAUNCH_HEIGHT_MM = 304.143;
    private static final double TARGET_HEIGHT_ABOVE_TAG_MM = 152.4;
    private static final double LAUNCH_ANGLE_DEG = 54.0;
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);
    private static final double GRAVITY_MM_S2 = 9810.0;
    private static final double MAX_MOTOR_RPM = 6000.0;
    private static final double FIELD_SIZE_MM = 3657.6;
    
    // Bump resistance constants
    private static final double BUMP_DETECTION_THRESHOLD = 50.0; // mm - TUNE THIS
    private static final double BUMP_RESISTANCE_GAIN = 0.8; // 0-1, higher = more aggressive resistance
    private static final double VELOCITY_SMOOTHING = 0.7; // Low-pass filter for velocity
    
    // DECODE Season AprilTag positions
    private static final double[][] TAG_POSITIONS = {
        {0.0, FIELD_SIZE_MM / 2},
        {0.0, 1828.8},
        {0.0, 1828.8},
        {0.0, 1828.8},
        {0.0, -FIELD_SIZE_MM / 2}
    };
    
    // Robot state
    private double robotX = 0, robotY = 0, robotHeading = 0;
    private int lastXodoPos = 0, lastYodoPos = 0;
    private static final double COUNTS_PER_MM = 1.0;
    
    // Bump resistance state
    private boolean bumpResistanceEnabled = true;
    private double expectedVelX = 0, expectedVelY = 0;
    private double actualVelX = 0, actualVelY = 0;
    private double smoothedVelX = 0, smoothedVelY = 0;
    private long lastLoopTime = 0;
    private double lastRobotX = 0, lastRobotY = 0;
    
    // Control state
    private boolean isBlueAlliance = true;
    private boolean turretEnabled = true;
    private boolean spatulaUp = false;
    private double launcherSpeedMultiplier = 1.0;
    private int targetTagId = 20;
    
    // Button debounce
    private boolean lastStartButton = false;
    private boolean lastBackButton = false;
    private boolean lastGuideButton = false;
    
    @Override
    public void runOpMode() {
        initHardware();
        
        telemetry.addData("Status", "Initialized - DECODE Auto-Aiming Ready!");
        telemetry.addData("Controls", "DPad L/R: Alliance | X: Launch | Y: Intake");
        telemetry.addData("NEW", "Guide Button: Toggle Bump Resistance");
        telemetry.update();
        
        waitForStart();
        lastLoopTime = System.currentTimeMillis();
        
        while (opModeIsActive()) {
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastLoopTime) / 1000.0;
            lastLoopTime = currentTime;
            
            // Update odometry and calculate velocities
            updateOdometry(deltaTime);
            
            // Handle controls
            handleDriveControls();
            handleAllianceSelection();
            handleLauncherControls();
            handleIntakeControls();
            handleTurretControls();
            handleModeToggles();
            
            // Apply bump resistance if enabled
            if (bumpResistanceEnabled) {
                applyBumpResistance();
            }
            
            // Auto-aiming logic runs continuously
            performAutoAiming();
            
            // Display telemetry
            updateTelemetry();
        }
    }
    
    private void initHardware() {
        // Initialize drive motors
        frontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        
        launchMotor = hardwareMap.get(DcMotor.class, "LaunchMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "IntakeMotor");
        
        xodo = hardwareMap.get(DcMotor.class, "Xodo");
        yodo = hardwareMap.get(DcMotor.class, "Yodo");
        
        // Initialize servos
        spinSpinServo = hardwareMap.get(Servo.class, "SpinSpinServo");
        spatulaServo = hardwareMap.get(Servo.class, "SpatulaServo");
        stopServo = hardwareMap.get(Servo.class, "StopServo");
        
        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        
        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        
        // Reset encoders
        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    private void handleDriveControls() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.right_stick_x;
        double rotate = 0;
        
        if (gamepad1.left_bumper) rotate = -0.6;
        if (gamepad1.right_bumper) rotate = 0.6;
        
        // Store expected velocity for bump detection
        expectedVelX = strafe * 1000.0; // Approximate mm/s (CALIBRATE)
        expectedVelY = forward * 1000.0;
        
        // Calculate wheel powers
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;
        
        // Normalize powers
        double maxPower = Math.max(Math.abs(frontLeftPower), 
                          Math.max(Math.abs(frontRightPower),
                          Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
        
        // Apply powers
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
    
    private void applyBumpResistance() {
        // Calculate velocity error (actual vs expected)
        double velErrorX = actualVelX - expectedVelX;
        double velErrorY = actualVelY - expectedVelY;
        double velErrorMagnitude = Math.sqrt(velErrorX * velErrorX + velErrorY * velErrorY);
        
        // Only resist if error exceeds threshold (indicating a bump)
        if (velErrorMagnitude > BUMP_DETECTION_THRESHOLD) {
            // Calculate resistance force (opposite to unexpected velocity)
            double resistX = -velErrorX * BUMP_RESISTANCE_GAIN / 1000.0; // Normalize to motor power
            double resistY = -velErrorY * BUMP_RESISTANCE_GAIN / 1000.0;
            
            // Convert to wheel powers (mecanum drive inverse kinematics)
            double resistFL = resistY + resistX;
            double resistFR = resistY - resistX;
            double resistBL = resistY - resistX;
            double resistBR = resistY + resistX;
            
            // Add resistance to current motor powers
            double currentFL = frontLeftMotor.getPower();
            double currentFR = frontRightMotor.getPower();
            double currentBL = backLeftMotor.getPower();
            double currentBR = backRightMotor.getPower();
            
            frontLeftMotor.setPower(Range.clip(currentFL + resistFL, -1.0, 1.0));
            frontRightMotor.setPower(Range.clip(currentFR + resistFR, -1.0, 1.0));
            backLeftMotor.setPower(Range.clip(currentBL + resistBL, -1.0, 1.0));
            backRightMotor.setPower(Range.clip(currentBR + resistBR, -1.0, 1.0));
        }
    }
    
    private void handleAllianceSelection() {
        if (gamepad1.dpad_left) {
            isBlueAlliance = true;
            targetTagId = 20;
        }
        if (gamepad1.dpad_right) {
            isBlueAlliance = false;
            targetTagId = 24;
        }
    }
    
    private void handleLauncherControls() {
        if (gamepad1.x) {
            launch();
        }
        
        if (gamepad1.dpad_up) {
            launcherSpeedMultiplier += 0.05;
            launcherSpeedMultiplier = Math.min(launcherSpeedMultiplier, 1.5);
        }
        if (gamepad1.dpad_down) {
            launcherSpeedMultiplier -= 0.05;
            launcherSpeedMultiplier = Math.max(launcherSpeedMultiplier, 0.5);
        }
    }
    
    private void handleIntakeControls() {
        if (gamepad1.y) {
            intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0.0);
        }
        
        if (gamepad1.b) {
            stopServo.setPosition(0.0);
        } else {
            stopServo.setPosition(1.0);
        }
    }
    
    private void handleTurretControls() {
        boolean currentBackButton = gamepad1.back;
        if (currentBackButton && !lastBackButton) {
            turretEnabled = !turretEnabled;
            if (!turretEnabled) {
                spinSpinServo.setPosition(0.5);
            }
        }
        lastBackButton = currentBackButton;
    }
    
    private void handleModeToggles() {
        // Start Button - Toggle spatula
        boolean currentStartButton = gamepad1.start;
        if (currentStartButton && !lastStartButton) {
            spatulaUp = !spatulaUp;
            spatulaServo.setPosition(spatulaUp ? 1.0 : 0.0);
        }
        lastStartButton = currentStartButton;
        
        // Guide Button - Toggle bump resistance
        boolean currentGuideButton = gamepad1.guide;
        if (currentGuideButton && !lastGuideButton) {
            bumpResistanceEnabled = !bumpResistanceEnabled;
        }
        lastGuideButton = currentGuideButton;
        
        // A Button - Return to center
        if (gamepad1.a) {
            moveToCenter();
        }
    }
    
    private void performAutoAiming() {
        if (!turretEnabled) return;
        
        LLResult result = limelight.getLatestResult();
        
        if (result != null && result.isValid() && result.getFiducialResults().size() > 0) {
            for (var fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetTagId) {
                    double limelightDistance = calculateDistanceFromLimelight(fiducial);
                    double odoDistance = confirmPositionWithOdometry(targetTagId);
                    double finalDistance = (limelightDistance + odoDistance) / 2.0;
                    
                    double requiredVelocity = calculateLaunchVelocity(finalDistance);
                    double requiredRPM = velocityToRPM(requiredVelocity) * launcherSpeedMultiplier;
                    
                    aimTurretAtTag(fiducial);
                    setLauncherSpeed(requiredRPM);
                    
                    break;
                }
            }
        }
    }
    
    private void updateOdometry(double deltaTime) {
        int currentXodoPos = xodo.getCurrentPosition();
        int currentYodoPos = yodo.getCurrentPosition();
        
        int deltaX = currentXodoPos - lastXodoPos;
        int deltaY = currentYodoPos - lastYodoPos;
        
        robotX += deltaX / COUNTS_PER_MM;
        robotY += deltaY / COUNTS_PER_MM;
        
        // Calculate actual velocity from odometry
        if (deltaTime > 0) {
            actualVelX = (robotX - lastRobotX) / deltaTime;
            actualVelY = (robotY - lastRobotY) / deltaTime;
            
            // Apply low-pass filter for smoother velocity reading
            smoothedVelX = VELOCITY_SMOOTHING * smoothedVelX + (1 - VELOCITY_SMOOTHING) * actualVelX;
            smoothedVelY = VELOCITY_SMOOTHING * smoothedVelY + (1 - VELOCITY_SMOOTHING) * actualVelY;
            
            actualVelX = smoothedVelX;
            actualVelY = smoothedVelY;
        }
        
        lastRobotX = robotX;
        lastRobotY = robotY;
        lastXodoPos = currentXodoPos;
        lastYodoPos = currentYodoPos;
    }
    
    private double calculateDistanceFromLimelight(com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial) {
        double area = fiducial.getTargetArea();
        double distance = 5000.0 / Math.sqrt(area);
        return distance;
    }
    
    private double confirmPositionWithOdometry(int tagId) {
        if (tagId < 20 || tagId > 24) return 0;
        
        double[] tagPos = TAG_POSITIONS[tagId - 20];
        double dx = tagPos[0] - robotX;
        double dy = tagPos[1] - robotY;
        
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    private double calculateLaunchVelocity(double horizontalDistance) {
        double deltaY = TARGET_HEIGHT_ABOVE_TAG_MM - LAUNCH_HEIGHT_MM;
        double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);
        double tanTheta = Math.tan(LAUNCH_ANGLE_RAD);
        
        double numerator = GRAVITY_MM_S2 * horizontalDistance * horizontalDistance;
        double denominator = 2 * cosTheta * cosTheta * (horizontalDistance * tanTheta - deltaY);
        
        if (denominator <= 0) return 0;
        
        return Math.sqrt(numerator / denominator);
    }
    
    private double velocityToRPM(double velocityMmPerS) {
        double wheelDiameterMm = 100.0;
        double wheelCircumferenceMm = Math.PI * wheelDiameterMm;
        
        double rps = velocityMmPerS / wheelCircumferenceMm;
        return rps * 60.0;
    }
    
    private void aimTurretAtTag(com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial) {
        double tx = fiducial.getTargetXDegrees();
        
        double currentPos = spinSpinServo.getPosition();
        double angleOffset = tx / 1800.0;
        
        double newPos = Range.clip(currentPos + angleOffset, 0.0, 1.0);
        spinSpinServo.setPosition(newPos);
    }
    
    private void setLauncherSpeed(double rpm) {
        double power = Range.clip(rpm / MAX_MOTOR_RPM, 0.0, 1.0);
        launchMotor.setPower(power);
    }
    
    private void launch() {
        spatulaServo.setPosition(1.0);
        sleep(300);
        spatulaServo.setPosition(0.0);
    }
    
    private void moveToCenter() {
        double kP = 0.01;
        double xError = -robotX;
        double yError = -robotY;
        
        double xPower = Range.clip(xError * kP, -0.5, 0.5);
        double yPower = Range.clip(yError * kP, -0.5, 0.5);
        
        frontLeftMotor.setPower(yPower + xPower);
        frontRightMotor.setPower(yPower - xPower);
        backLeftMotor.setPower(yPower - xPower);
        backRightMotor.setPower(yPower + xPower);
    }
    
    private void updateTelemetry() {
        telemetry.addData("Alliance", isBlueAlliance ? "BLUE (Tag 20)" : "RED (Tag 24)");
        telemetry.addData("Turret", turretEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Bump Resist", bumpResistanceEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Launch Speed", "%.0f%%", launcherSpeedMultiplier * 100);
        telemetry.addData("Robot Pos (mm)", "X: %.0f, Y: %.0f", robotX, robotY);
        telemetry.addData("Velocity (mm/s)", "X: %.0f, Y: %.0f", actualVelX, actualVelY);
        telemetry.addData("Spatula", spatulaUp ? "UP" : "DOWN");
        telemetry.update();
    }
}

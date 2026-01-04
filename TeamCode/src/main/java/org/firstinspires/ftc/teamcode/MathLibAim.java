package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Auto-Aiming (Using Math-Lib Gradient)")
public class MathLibAim extends LinearOpMode {
    
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
    private static final double TARGET_HEIGHT_ABOVE_TAG_MM = 152.4; // 6 inches
    private static final double LAUNCH_ANGLE_DEG = 54.0;
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);
    private static final double GRAVITY_MM_S2 = 9810.0;
    private static final double MAX_MOTOR_RPM = 6000.0;
    private static final double FIELD_SIZE_MM = 3657.6; // 12 feet in mm
    
    // DECODE Season AprilTag positions (in mm from field center)
    private static final double[][] TAG_POSITIONS = {
        {0.0, FIELD_SIZE_MM / 2},      // Tag 20 - Blue Alliance Goal
        {0.0, 1828.8},                 // Tag 21 - Neutral (0, 6ft)
        {0.0, 1828.8},                 // Tag 22 - Neutral (0, 6ft)
        {0.0, 1828.8},                 // Tag 23 - Neutral (0, 6ft)
        {0.0, -FIELD_SIZE_MM / 2}      // Tag 24 - Red Alliance Goal
    };
    
    // Robot state
    private double robotX = 0, robotY = 0, robotHeading = 0;
    private int lastXodoPos = 0, lastYodoPos = 0;
    private static final double COUNTS_PER_MM = 1.0; // CALIBRATE THIS!
    
    // Control state
    private boolean isBlueAlliance = true;
    private boolean turretEnabled = true;
    private boolean spatulaUp = false;
    private double launcherSpeedMultiplier = 1.0;
    private int targetTagId = 20; // Default to blue
    
    // Button debounce
    private boolean lastStartButton = false;
    private boolean lastBackButton = false;
    
    @Override
    public void runOpMode() {
        initHardware();
        
        telemetry.addData("Status", "Initialized - DECODE Auto-Aiming Ready!");
        telemetry.addData("Controls", "DPad L/R: Alliance | X: Launch | Y: Intake");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            // Update odometry
            updateOdometry();
            
            // Handle controls
            handleDriveControls();
            handleAllianceSelection();
            handleLauncherControls();
            handleIntakeControls();
            handleTurretControls();
            handleModeToggles();
            
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
        
        // Set motor directions (adjust based on your robot)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        
        // Reset encoders
        xodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yodo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yodo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    
    private void handleDriveControls() {
        double forward = -gamepad1.left_stick_y;  // Front/Back
        double strafe = gamepad1.right_stick_x;   // Left/Right
        double rotate = 0;
        
        // Rotation with bumpers
        if (gamepad1.left_bumper) rotate = -0.6;
        if (gamepad1.right_bumper) rotate = 0.6;
        
        // Calculate wheel powers for mecanum drive
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
        // X Button - Launch
        if (gamepad1.x) {
            launch();
        }
        
        // D-Pad Up/Down - Adjust launch speed
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
        // Y Button - Intake
        if (gamepad1.y) {
            intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0.0);
        }
        
        // B Button - Ball stopper servo
        if (gamepad1.b) {
            stopServo.setPosition(0.0); // Open
        } else {
            stopServo.setPosition(1.0); // Closed
        }
    }
    
    private void handleTurretControls() {
        // Back Button - Toggle turret (faces backwards when off)
        boolean currentBackButton = gamepad1.back;
        if (currentBackButton && !lastBackButton) {
            turretEnabled = !turretEnabled;
            if (!turretEnabled) {
                spinSpinServo.setPosition(0.5); // Face backwards (adjust as needed)
            }
        }
        lastBackButton = currentBackButton;
    }
    
    private void handleModeToggles() {
        // Left/Right triggers reserved for future servo functions
        
        // Start Button - Toggle spatula
        boolean currentStartButton = gamepad1.start;
        if (currentStartButton && !lastStartButton) {
            spatulaUp = !spatulaUp;
            spatulaServo.setPosition(spatulaUp ? 1.0 : 0.0);
        }
        lastStartButton = currentStartButton;
        
        // A Button - Return to center
        if (gamepad1.a) {
            moveToCenter();
        }
    }
    
    private void performAutoAiming() {
        if (!turretEnabled) return;
        
        LLResult result = limelight.getLatestResult();
        
        if (result != null && result.isValid() && result.getFiducialResults().size() > 0) {
            // Find the target tag
            for (var fiducial : result.getFiducialResults()) {
                if (fiducial.getFiducialId() == targetTagId) {
                    // Calculate distance using both methods
                    double limelightDistance = calculateDistanceFromLimelight(fiducial);
                    double odoDistance = confirmPositionWithOdometry(targetTagId);
                    double finalDistance = (limelightDistance + odoDistance) / 2.0;
                    
                    // Calculate required launch velocity and RPM
                    double requiredVelocity = calculateLaunchVelocity(finalDistance);
                    double requiredRPM = velocityToRPM(requiredVelocity) * launcherSpeedMultiplier;
                    
                    // Aim turret and set launcher speed
                    aimTurretAtTag(fiducial);
                    setLauncherSpeed(requiredRPM);
                    
                    break;
                }
            }
        }
    }
    
    private void updateOdometry() {
        int currentXodoPos = xodo.getCurrentPosition();
        int currentYodoPos = yodo.getCurrentPosition();
        
        int deltaX = currentXodoPos - lastXodoPos;
        int deltaY = currentYodoPos - lastYodoPos;
        
        robotX += deltaX / COUNTS_PER_MM;
        robotY += deltaY / COUNTS_PER_MM;
        
        lastXodoPos = currentXodoPos;
        lastYodoPos = currentYodoPos;
    }
    
    private double calculateDistanceFromLimelight(com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial) {
        // Get target pose data
        double tx = fiducial.getTargetXDegrees();
        double ty = fiducial.getTargetYDegrees();
        
        // Use Limelight's camera pose if available
        // Otherwise estimate from pixel measurements
        // CALIBRATE THIS based on your Limelight mounting and calibration
        double area = fiducial.getTargetArea();
        
        // Rough distance estimation (calibrate this formula)
        double distance = 5000.0 / Math.sqrt(area); // Example formula
        
        return distance;
    }
    
    private double confirmPositionWithOdometry(int tagId) {
        // Get tag position from lookup table
        if (tagId < 20 || tagId > 24) return 0;
        
        double[] tagPos = TAG_POSITIONS[tagId - 20];
        double dx = tagPos[0] - robotX;
        double dy = tagPos[1] - robotY;
        
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    private double calculateLaunchVelocity(double horizontalDistance) {
        // Projectile motion: v₀ = √[g·d² / (2·cos²(θ)·(d·tan(θ) - Δy))]
        double deltaY = TARGET_HEIGHT_ABOVE_TAG_MM - LAUNCH_HEIGHT_MM;
        double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);
        double tanTheta = Math.tan(LAUNCH_ANGLE_RAD);
        
        double numerator = GRAVITY_MM_S2 * horizontalDistance * horizontalDistance;
        double denominator = 2 * cosTheta * cosTheta * (horizontalDistance * tanTheta - deltaY);
        
        if (denominator <= 0) return 0; // Safety check
        
        return Math.sqrt(numerator / denominator);
    }
    
    private double velocityToRPM(double velocityMmPerS) {
        // CALIBRATE: Measure your launch wheel diameter
        double wheelDiameterMm = 100.0;
        double wheelCircumferenceMm = Math.PI * wheelDiameterMm;
        
        double rps = velocityMmPerS / wheelCircumferenceMm;
        return rps * 60.0;
    }
    
    private void aimTurretAtTag(com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial) {
        double tx = fiducial.getTargetXDegrees();
        
        // Convert angle to servo position (5 turn servo = 1800° range)
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
        // Launch sequence: feed ball through spatula
        spatulaServo.setPosition(1.0);
        sleep(300);
        spatulaServo.setPosition(0.0);
    }
    
    private void moveToCenter() {
        // Simple proportional control to move robot to center (0, 0)
        double kP = 0.01; // Tune this
        double xError = -robotX;
        double yError = -robotY;
        
        double xPower = Range.clip(xError * kP, -0.5, 0.5);
        double yPower = Range.clip(yError * kP, -0.5, 0.5);
        
        // Apply powers for 1 second
        frontLeftMotor.setPower(yPower + xPower);
        frontRightMotor.setPower(yPower - xPower);
        backLeftMotor.setPower(yPower - xPower);
        backRightMotor.setPower(yPower + xPower);
    }
    
    private void updateTelemetry() {
        telemetry.addData("Alliance", isBlueAlliance ? "BLUE (Tag 20)" : "RED (Tag 24)");
        telemetry.addData("Turret", turretEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Launch Speed", "%.0f%%", launcherSpeedMultiplier * 100);
        telemetry.addData("Robot Pos (mm)", "X: %.0f, Y: %.0f", robotX, robotY);
        telemetry.addData("Spatula", spatulaUp ? "UP" : "DOWN");
        telemetry.update();
    }
}

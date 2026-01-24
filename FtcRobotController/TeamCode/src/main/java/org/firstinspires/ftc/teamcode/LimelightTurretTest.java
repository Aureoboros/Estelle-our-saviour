package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

@TeleOp(name = "Limelight Turret Test")
public class LimelightTurretTest extends LinearOpMode {

    // Servo constants
    private static final double SERVO_START_POS = 0.35;
    private static final double SERVO_RANGE_90 = 0.5;
    private static final double SERVO_FULL_ROTATION = 1.0; // Full servo range represents 360 degrees for continuous rotation
    private static final double DEGREES_PER_SERVO_UNIT = 360.0; // Degrees per full servo range (calibrate this)

    // Tracking constants
    private static final double TX_TOLERANCE = 2.0;
    private static final double PROPORTIONAL_GAIN = 0.01;
    private static final double MAX_STEP = 0.02; // prevents snapping to 90°
    private static final double MAX_ANGLE_THRESHOLD = 90.0; // If angle > 90°, go the other way

    // Pipelines / tags
    private static final int PIPELINE_TAG_24 = 0;
    private static final int PIPELINE_TAG_20 = 1;
    private static final int TAG_24_ID = 24;
    private static final int TAG_20_ID = 20;

    // Distance calculation constants (CALIBRATE THESE FOR YOUR ROBOT)
    private static final double CAMERA_HEIGHT_MM = 200.0;  // Height of camera lens from ground
    private static final double CAMERA_ANGLE_DEG = 5.0;    // Camera tilt angle (positive = tilted up)
    private static final double TAG_HEIGHT_MM = 150.0;     // Height of AprilTag center from ground
    private static final double AREA_CALIBRATION_CONSTANT = 5000.0;  // Calibration constant for area-based distance

    private Servo spinSpinServo;
    private Limelight3A limelight;

    private double servoMin;
    private double servoMax;

    private boolean trackingTag24 = false;
    private boolean trackingTag20 = false;
    private int currentTargetTag = -1;
    private int currentPipeline = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        Gamepad current = new Gamepad();
        Gamepad previous = new Gamepad();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previous.copy(current);
            current.copy(gamepad1);

            boolean bPressed = current.b && !previous.b;
            boolean xPressed = current.x && !previous.x;
            boolean yPressed = current.y && !previous.y;

            // ---- TAG 24 ----
            if (bPressed) {
                trackingTag24 = !trackingTag24;
                trackingTag20 = false;

                if (trackingTag24) {
                    currentTargetTag = TAG_24_ID;
                    currentPipeline = PIPELINE_TAG_24;
                    limelight.pipelineSwitch(currentPipeline);
                } else {
                    currentTargetTag = -1;
                }
            }

            // ---- TAG 20 ----
            if (xPressed) {
                trackingTag20 = !trackingTag20;
                trackingTag24 = false;

                if (trackingTag20) {
                    currentTargetTag = TAG_20_ID;
                    currentPipeline = PIPELINE_TAG_20;
                    limelight.pipelineSwitch(currentPipeline);
                } else {
                    currentTargetTag = -1;
                }
            }

            // ---- RESET ----
            if (yPressed) {
                trackingTag24 = false;
                trackingTag20 = false;
                currentTargetTag = -1;
                currentPipeline = -1;
                spinSpinServo.setPosition(SERVO_START_POS);
            }

            if (currentTargetTag != -1) {
                updateTurretTracking();
            }

            displayTelemetry();
            telemetry.update();
        }
    }

    private void initializeHardware() {
        spinSpinServo = hardwareMap.get(Servo.class, "spinSpinServo");
        spinSpinServo.setPosition(SERVO_START_POS);

        servoMin = Range.clip(SERVO_START_POS - SERVO_RANGE_90, 0.0, 1.0);
        servoMax = Range.clip(SERVO_START_POS + SERVO_RANGE_90, 0.0, 1.0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        // Verify Limelight connection
        telemetry.addLine("--- Limelight Status ---");
        telemetry.addData("Limelight Connected", limelight.isConnected());
        telemetry.addData("Limelight Running", limelight.isRunning());
        telemetry.update();
        sleep(500); // Give Limelight time to initialize
    }

    private void updateTurretTracking() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (f.getFiducialId() == currentTargetTag) {
                target = f;
                break;
            }
        }
        if (target == null) return;

        double tx = target.getTargetXDegrees();
        if (Math.abs(tx) <= TX_TOLERANCE) return;

        // Calculate the shortest path to the target
        // If the angle is more than 90 degrees, move the other way (360 - angle)
        double adjustmentAngle = tx;
        
        if (Math.abs(tx) > MAX_ANGLE_THRESHOLD) {
            // The tag is more than 90° away in current direction
            // Go the other way: if tx is +120°, go -240° instead (or vice versa)
            if (tx > 0) {
                adjustmentAngle = tx - 360.0; // e.g., 120° becomes -240°
            } else {
                adjustmentAngle = tx + 360.0; // e.g., -120° becomes +240°
            }
        }
        
        // Convert angle to servo adjustment
        // Negative because we want to move toward the target (opposite of offset)
        double adjustment = Range.clip(-adjustmentAngle * PROPORTIONAL_GAIN, -MAX_STEP, MAX_STEP);
        double currentPos = spinSpinServo.getPosition();
        double newPos = currentPos + adjustment;
        
        // Handle wraparound for continuous rotation servo
        // If we go past the limits, wrap around
        if (newPos > 1.0) {
            newPos = newPos - 1.0;
        } else if (newPos < 0.0) {
            newPos = 1.0 + newPos;
        }

        spinSpinServo.setPosition(Range.clip(newPos, 0.0, 1.0));
    }

    private void displayTelemetry() {
        telemetry.addLine("===== LIMELIGHT TURRET =====");

        if (currentTargetTag == -1) {
            telemetry.addData("Tracking", "NONE");
        } else {
            telemetry.addData("Tracking Tag", currentTargetTag);
            telemetry.addData("Pipeline", currentPipeline);
        }

        telemetry.addData("Servo Pos", "%.3f", spinSpinServo.getPosition());
        telemetry.addData("Active Zone", "%.2f → %.2f", servoMin, servoMax);
        
        telemetry.addLine();
        telemetry.addLine("--- Limelight Status ---");
        telemetry.addData("Limelight Connected", limelight.isConnected());
        telemetry.addData("Limelight Running", limelight.isRunning());

        telemetry.addLine();
        telemetry.addLine("--- Limelight Status ---");
        telemetry.addData("Limelight Connected", limelight.isConnected());
        telemetry.addData("Limelight Running", limelight.isRunning());

        LLResult r = limelight.getLatestResult();
        if (r == null) {
            telemetry.addData("Result", "NULL - No data from Limelight");
            telemetry.addLine("Check: USB connection, power, Control Hub config");
        } else {
            telemetry.addData("Result Valid", r.isValid());
            telemetry.addData("Pipeline Index", r.getPipelineIndex());
            telemetry.addData("Timestamp", "%.3f", r.getTimestamp());
            telemetry.addData("Targeting Latency", "%.1f ms", r.getTargetingLatency());
            int fiducialCount = r.getFiducialResults().size();
            telemetry.addData("AprilTags Detected", fiducialCount);
            if (r.isValid() && fiducialCount > 0) {
                telemetry.addLine();
                telemetry.addLine("--- AprilTag Data ---");
                for (LLResultTypes.FiducialResult fiducial : r.getFiducialResults()) {
                    int tagId = fiducial.getFiducialId();
                    telemetry.addData("Tag ID", tagId);
                    telemetry.addData("TX (horizontal)", "%.2f°", fiducial.getTargetXDegrees());
                    telemetry.addData("TY (vertical)", "%.2f°", fiducial.getTargetYDegrees());
                    telemetry.addData("Target Area", "%.4f", fiducial.getTargetArea());
                    
                    // Calculate and display distance
                    double distance = calculateDistanceFromLimelight(fiducial);
                    telemetry.addData("Distance", "%.0f mm (%.1f in)", distance, distance / 25.4);
                    
                    // Show if this is the tag we're tracking
                    if (tagId == currentTargetTag) {
                        telemetry.addData("Status", "✓ TRACKING");
                    }
                    telemetry.addLine();
                }
            } else if (r.isValid()) {
                telemetry.addLine();
                telemetry.addData("Status", "No AprilTags in view");
                telemetry.addLine("Aim camera at AprilTag 20 or 24");
            } else {
                telemetry.addLine();
                telemetry.addData("Status", "Result invalid");
                telemetry.addLine("Check pipeline settings in Limelight");
            }
        }
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addLine("B = Toggle Track Tag 24 (Red)");
        telemetry.addLine("X = Toggle Track Tag 20 (Blue)");
        telemetry.addLine("Y = Reset to Center");
    }

    /**
     * Calculates the distance to an AprilTag using two methods:
     * 1. TY-based (primary): Uses vertical angle trigonometry for more accuracy
     * 2. Area-based (fallback): Uses target area when TY method is invalid
     * 
     * @param fiducial The fiducial result from Limelight
     * @return Distance to tag in millimeters
     */
    private double calculateDistanceFromLimelight(LLResultTypes.FiducialResult fiducial) {
        // Method 1: Use target area (requires calibration)
        double area = fiducial.getTargetArea();
        
        // Method 2: Use TY (vertical angle) for more accurate distance
        // This uses the known height difference between camera and tag
        double ty = fiducial.getTargetYDegrees();
        
        // Calculate distance using trigonometry
        // distance = (tagHeight - cameraHeight) / tan(cameraAngle + ty)
        double angleToTargetRad = Math.toRadians(CAMERA_ANGLE_DEG + ty);
        
        double distanceFromTY = 0.0;
        if (Math.abs(angleToTargetRad) > 0.01) {  // Avoid division by zero
            distanceFromTY = Math.abs((TAG_HEIGHT_MM - CAMERA_HEIGHT_MM) / Math.tan(angleToTargetRad));
        }
        
        // Method 3: Use area-based estimation as fallback
        // Formula: distance = k / sqrt(area), where k is calibration constant
        // CALIBRATE: Measure area at known distance, then k = distance * sqrt(area)
        double distanceFromArea = AREA_CALIBRATION_CONSTANT / Math.sqrt(Math.max(area, 0.0001));
        
        // Use TY-based distance if valid, otherwise fall back to area
        double distance;
        if (distanceFromTY > 100.0 && distanceFromTY < 5000.0) {
            // TY-based distance seems reasonable
            distance = distanceFromTY;
        } else {
            // Fall back to area-based
            distance = distanceFromArea;
        }
        
        return distance;
    }
}

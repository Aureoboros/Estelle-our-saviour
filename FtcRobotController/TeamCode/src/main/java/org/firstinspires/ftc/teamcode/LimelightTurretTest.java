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

    // Tracking constants
    private static final double TX_TOLERANCE = 2.0;
    private static final double PROPORTIONAL_GAIN = 0.01;
    private static final double MAX_STEP = 0.02; // prevents snapping to 90°

    // Pipelines / tags
    private static final int PIPELINE_TAG_24 = 0;
    private static final int PIPELINE_TAG_20 = 1;
    private static final int TAG_24_ID = 24;
    private static final int TAG_20_ID = 20;

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

        double adjustment = Range.clip(-tx * PROPORTIONAL_GAIN, -MAX_STEP, MAX_STEP);
        double newPos = spinSpinServo.getPosition() + adjustment;

        spinSpinServo.setPosition(Range.clip(newPos, servoMin, servoMax));
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
}

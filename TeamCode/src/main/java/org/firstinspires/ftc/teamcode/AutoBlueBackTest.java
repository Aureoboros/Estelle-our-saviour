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
    // Blue alliance spike marks (right side of field, mirrored from red)
    private static final double BLUE_LOWER_SPIKE_X = -3.0;
    private static final double BLUE_LOWER_SPIKE_Y = -3.0;

    private static final double BLUE_MIDDLE_SPIKE_X = -3.0;  // Middle spike mark (mirrored)
    private static final double BLUE_MIDDLE_SPIKE_Y = -1.0;  // Below center line

    private static final double BLUE_TOP_SPIKE_X = -3.0;     // Top spike mark (same X)
    private static final double BLUE_TOP_SPIKE_Y = 1.0;      // Above center line

    // Shooting position (at field center for AprilTag alignment)
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

// Actual code starts here: DEBUG LIKE HELL BEYOND THIS LINE

public class actualCodeLol {
    private void intakeGateBalls(double duration) {
        driveToPosition(-2.0, -1.0);
        driveToPosition(-5.0, 1.0); // OG values: (-5.5, -0.75)
        drive.turn(Math.toRadians(60));
        intakeMotor.setPower(INTAKE_POWER);
        sleep((long)(duration * 4000));
        intakeMotor.setPower(0);
      }

    private void intakeSpikeBalls(double duration, double spike_x, double spike_y, double angle) {
        // SOMEBODY NEEDS TO CHECK IF SPIKE_X AND Y ARE IN INCHES OR SOMETHING ELSE
        driveToPosition(spike_x, spike_y);
        drive.turn(Math.toRadians(angle)); // CHECK PLEASE
        intakeMotor.setPower(INTAKE_POWER);
        driveToPosition(spike_x - 2.0, spike_y);
        intakeMotor.setPower(0);
        driveToPosition(spike_x + 2.0, spike_y);
      }

    private void intakeSpikeBalls(double duration, double spike_x, double spike_y, double angle) {
        // SOMEBODY NEEDS TO CHECK IF SPIKE_X AND Y ARE IN INCHES OR SOMETHING ELSE
        driveToPosition(-2.5, -1.0)
        driveToPosition(spike_x, spike_y);
        drive.turn(Math.toRadians(angle)); // CHECK PLEASE
        intakeMotor.setPower(INTAKE_POWER);
        driveToPosition(spike_x - 2.0, spike_y);
        intakeMotor.setPower(0);
        driveToPosition(spike_x + 2.0, spike_y);
      }
    
// Use preloaded balls then go to middle spike then shoot again then "open gate/load then shoot"
// Then do the same thing a few times then go to tops spike and shoot then go to bottom spike and shoot

    // Stopping the defining of functions and starting to cell them
    // Do I needed to add something here before I call stuff???
    
       shootBalls();
       intakeSpikeBalls(-2.5, -1.0); // THIS IS ASSUMING IT WORKS IN TICKS AND I DON'T EVEN HAVE DURATION OR ANGLE IN HERE
       shootBalls();
       
       // FINISH
    
    
}

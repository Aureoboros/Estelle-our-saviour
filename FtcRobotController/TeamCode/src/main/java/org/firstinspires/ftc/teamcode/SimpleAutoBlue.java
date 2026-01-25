package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous(name="SimpleAutoBlue", group="Autonomous")
public class SimpleAutoBlue extends LinearOpMode {
    private static final double FRONT_LEFT_POWER = 0.75;
    private static final double BACK_LEFT_POWER = 0.75;
    private static final double FRONT_RIGHT_POWER = 0.75;
    private static final double BACK_RIGHT_POWER = 0.75;
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private DcMotor intakeMotor, launchMotor;
    private Servo spatulaServo, stopServo, spinSpinServo;


    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();
        if (isStopRequested()) return;
        frontLeftMotor.setPower(FRONT_LEFT_POWER);
        frontRightMotor.setPower(FRONT_RIGHT_POWER);
        backLeftMotor.setPower(BACK_LEFT_POWER);
        backRightMotor.setPower(BACK_RIGHT_POWER);
        sleep(50);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        launchBalls(4);
        frontLeftMotor.setPower(FRONT_LEFT_POWER);
        frontRightMotor.setPower(FRONT_RIGHT_POWER);
        backLeftMotor.setPower(BACK_LEFT_POWER);
        backRightMotor.setPower(BACK_RIGHT_POWER);
        sleep(300);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);


    }
    private void initializeHardware() {
        // Initialize drive motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Initialize other motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        stopServo = hardwareMap.get(Servo.class, "stopServo");
        spatulaServo = hardwareMap.get(Servo.class, "spatulaServo");
        spinSpinServo = hardwareMap.get(Servo.class, "spinSpinServo");

        stopServo.setPosition(1.0); // Closed
        spatulaServo.setPosition(1.0); // Down
        spinSpinServo.setPosition(0.865); // Set turret to initial position

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setPower(0.9);
    }
    private void launchBalls(int count) {
        // Turn intake motor ON in reverse direction to feed balls
        // Negative power for reverse direction to push balls forward
        intakeMotor.setPower(-1.0);
        
        for (int i = 0; i < count; i++) {
            // Open stopper to allow ball through
            stopServo.setPosition(1.0);
            sleep(1000);
            
            // Close stopper to stop other balls from going under the spatula
            stopServo.setPosition(0.5);
            sleep(1000);
            
            // Actuate spatula to push ball
            spatulaServo.setPosition(0.0);
            sleep(1000);
            spatulaServo.setPosition(1.0);
            sleep(1000);
            
            // Keep intake running between balls to feed next ball
            // Only turn off after last ball
            if (i < count - 1) {
                // Keep intake running to feed next ball
                sleep(500); // Brief pause to allow ball to settle
            }
            launchMotor.setPower(0.95);
        }
        
        // Turn intake motor OFF after all balls are launched
        intakeMotor.setPower(0);
    }

}

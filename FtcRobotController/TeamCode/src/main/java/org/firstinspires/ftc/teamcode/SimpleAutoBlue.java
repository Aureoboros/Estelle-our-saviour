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
    private DcMotor launchMotor;
    private Servo spatulaServo, stopServo;


    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();
        if (isStopRequested()) return;
        frontLeftMotor.setPower(FRONT_LEFT_POWER);
        frontRightMotor.setPower(FRONT_RIGHT_POWER);
        backLeftMotor.setPower(BACK_LEFT_POWER);
        backRightMotor.setPower(BACK_RIGHT_POWER);
        sleep(200);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        launchBalls(3);
        frontLeftMotor.setPower(FRONT_LEFT_POWER);
        frontRightMotor.setPower(FRONT_RIGHT_POWER);
        backLeftMotor.setPower(BACK_LEFT_POWER);
        backRightMotor.setPower(BACK_RIGHT_POWER);
        sleep(3000);
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
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        stopServo = hardwareMap.get(Servo.class, "stopServo");
        spatulaServo = hardwareMap.get(Servo.class, "spatulaServo");

        stopServo.setPosition(1.0); // Closed
        spatulaServo.setPosition(0.0); // Down

        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setPower(0.8);
        }
    private void launchBalls(int count) {
        for (int i = 0; i < count; i++) {
            // Open stopper to allow ball through
            stopServo.setPosition(0.0);
            sleep(100);

            // Actuate spatula to push ball
            spatulaServo.setPosition(1.0);
            sleep(300);
            spatulaServo.setPosition(0.0);
            sleep(200);

            // Close stopper
            stopServo.setPosition(1.0);
            sleep(200);
        }
    }

    }

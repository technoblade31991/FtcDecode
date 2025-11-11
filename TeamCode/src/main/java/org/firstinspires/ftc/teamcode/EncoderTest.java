package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static java.lang.Thread.sleep;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name="EncodersTest")
public class EncoderTest extends LinearOpMode {
    DcMotor leftRearDrive;
    DcMotor rightRearDrive;
    DcMotor leftFrontDrive;
    DcMotor rightFrontDrive;
    private ElapsedTime runtime = new ElapsedTime();
    static final double MOTOR_TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 3.779;
    static final double COUNTS_PER_INCH = MOTOR_TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * 3.14159);
    static final double DRIVE_SPEED = 0.5;

//    ColorSensor color1;
//    DistanceSensor distance1;
//    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        leftRearDrive = hardwareMap.get(DcMotor.class, "motor1");
        rightRearDrive = hardwareMap.get(DcMotor.class, "motor2");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "motor3");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motor4");
//        color1 = hardwareMap.get(ColorSensor.class, "color1");
//        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Put initialization blocks here
        leftRearDrive.setDirection(DcMotor.Direction.REVERSE);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftRearDrive.setTargetPosition(5000);
//        rightRearDrive.setTargetPosition(5000);
//        encoderDrive()
        waitForStart();
        encoderDrive(DRIVE_SPEED, 24, 24, 5.0); // Drive FORWARD 24 inches
        sleep(500); // Pause for 500 milliseconds
        encoderDrive(DRIVE_SPEED, -12, -12, 3.0); // Drive BACKWARD 12 inches

//        leftRearDrive.setPower(0.5);
//        rightRearDrive.setPower(0.5);

    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        int newLeftFrontTarget;
        int newLeftRearTarget;
        int newRightFrontTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Calculate new target position in "ticks"
            // We add this to the current position, just in case we didn't reset
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newLeftRearTarget = leftRearDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newRightRearTarget = rightRearDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            // Set the target position for each motor
            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftRearDrive.setTargetPosition(newLeftRearTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightRearDrive.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION mode
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout timer and start motion
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftRearDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightRearDrive.setPower(Math.abs(speed));

            // Keep looping WHILE the OpMode is active AND any motor is still busy running to the target
            // AND we haven't timed out
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() || leftRearDrive.isBusy() || rightFrontDrive.isBusy() || rightRearDrive.isBusy())) {

                // Send telemetry feedback
                telemetry.addData("Path1", "Running to LF:%7d LR:%7d RF:%7d RR:%7d",
                        newLeftFrontTarget, newLeftRearTarget, newRightFrontTarget, newRightRearTarget);
                telemetry.addData("Path2", "Running at LF:%7d LR:%7d RF:%7d RR:%7d",
                        leftFrontDrive.getCurrentPosition(),
                        leftRearDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(),
                        rightRearDrive.getCurrentPosition());
                telemetry.update();
            }

            // --- Move Complete: STOP all motors ---
            leftFrontDrive.setPower(0);
            leftRearDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightRearDrive.setPower(0);

            // --- Reset the motor modes to default ---
            // This prepares them for TeleOp (if needed) or the next encoder move
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // sleep(100); // Optional pause after move
        }

    }
}

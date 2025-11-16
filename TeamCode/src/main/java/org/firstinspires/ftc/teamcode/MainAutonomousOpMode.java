package org.firstinspires.ftc.teamcode;

/* this is defined in MainTeleOpOpMode */
import static org.firstinspires.ftc.teamcode.MainTeleOpOpMode.NEW_ROBOT;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@Autonomous(name="Main Autonomous Op Mode", group="Linear OpMode")
public class MainAutonomousOpMode extends LinearOpMode {

    // Declare motors

    // Robot constants (update for your specific setup)
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // GoBILDA 5202/5203 motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;       // No gear reduction
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;       // 4” Mecanum wheels
    static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     DRIVE_SPEED             = 0.2;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    double     TARGET_DISTANCE_INCHES  = -9.25; // Move backward 9.25 inches

    private static final boolean SHOOT_ENABLED = true;

    private static final int NUM_BALLS = 4;
    @Override
    public void runOpMode() {
        Shooter shooter;
        if (SHOOT_ENABLED) {
            shooter = new Shooter();
            shooter.init(hardwareMap, gamepad2, telemetry, true, NEW_ROBOT);
        } else {
            shooter = null;
        }
        // Initialize hardware
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight  = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Reverse the right side if needed
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            encoderDrive(DRIVE_SPEED, TARGET_DISTANCE_INCHES);
            if (SHOOT_ENABLED) {

                shooter.launch_n_balls(NUM_BALLS);
            }
            encoderDrive(0.5, -5);
            strafeLeftInches(20, 0.5);
        }
    }

    public void encoderDrive(double speed, double inches) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Determine new target position
        int moveCounts = (int)(inches * COUNTS_PER_INCH);
        newFrontLeftTarget  = frontLeft.getCurrentPosition() + moveCounts;
        newFrontRightTarget = frontRight.getCurrentPosition() + moveCounts;
        newBackLeftTarget   = backLeft.getCurrentPosition() + moveCounts;
        newBackRightTarget  = backRight.getCurrentPosition() + moveCounts;

        // Set target position
        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        // Turn On RUN_TO_POSITION
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start motion
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        // Keep looping while still moving and within timeout
        while (opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() &&
                        backLeft.isBusy() && backRight.isBusy())) {
            telemetry.addData("Target", "%7d:%7d", newFrontLeftTarget, newFrontRightTarget);
            telemetry.addData("Current", "%7d:%7d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset to normal mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void strafeLeftInches(double inches, double power) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        // Strafing left: set target positions
        int flTarget = frontLeft.getCurrentPosition() - moveCounts;
        int frTarget = frontRight.getCurrentPosition() + moveCounts;
        int blTarget = backLeft.getCurrentPosition() + moveCounts;
        int brTarget = backRight.getCurrentPosition() - moveCounts;

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        while (opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() &&
                        backLeft.isBusy() && backRight.isBusy())) {
            telemetry.addData("Strafing Left", "In Progress");
            telemetry.update();
        }

        // Stop all motion
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Return to normal encoder mode
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Strafing", "Complete");
        telemetry.update();
    }
}
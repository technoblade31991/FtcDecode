package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp(name="Main Autonomous Op Mode", group="Linear OpMode")
public class MainAutonomousOpMode extends LinearOpMode {

    // Declare motors
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Robot constants (update for your specific setup)
    static final double     COUNTS_PER_MOTOR_REV    = 537.7;    // GoBILDA 5202/5203 motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;       // No gear reduction
    static final double     WHEEL_DIAMETER_INCHES   = 4.0;       // 4” Mecanum wheels
    static final double     COUNTS_PER_INCH         =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double     DRIVE_SPEED             = 0.2;
    static final double     TARGET_DISTANCE_INCHES  = 36.0;

    private static final boolean DRIVE_ENABLED = true;
    private static final boolean SHOOT_ENABLED = true;
    @Override
    public void runOpMode() {
        Shooter shooter;
        if (SHOOT_ENABLED) {
            shooter = new Shooter();
            shooter.init(hardwareMap, gamepad2, telemetry);
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
            if (SHOOT_ENABLED) {
                for (int i = 0; i < 3; i++) {
                    shooter.listen(true);
                    while (!shooter.isOff()) {
                        shooter.listen(false);
                    }
                }
            }

            encoderDrive(DRIVE_SPEED, TARGET_DISTANCE_INCHES, 5.0);
        }
    }

    public void encoderDrive(double speed, double inches, double timeoutS) {
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
}
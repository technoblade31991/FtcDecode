package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
@Disabled
@Autonomous(name="Pattern Autonomous Op Mode", group="Linear OpMode")
public class PatternAutonomousOpMode extends LinearOpMode {

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
        MecanumDrive drive = new MecanumDrive();
        drive.init(hardwareMap, telemetry);
        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drive.driveRelativeRobot(1, 0, 0, 1);
            sleep(2000);
            drive.driveRelativeRobot(-1, 0, 0, 1);
            sleep(2000);
        }
    }
}
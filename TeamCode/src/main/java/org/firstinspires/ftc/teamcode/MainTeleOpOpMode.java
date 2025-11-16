package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.DistanceSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp(name = "MainTeleOpOpModeCombinedAb11")
public class MainTeleOpOpMode extends LinearOpMode {

    private static boolean DRIVE_ENABLED = true;
    private static boolean SHOOT_ENABLED = true;
    private static boolean INTAKE_ENABLED = true;
    private static boolean DISTANCE_ENABLED = true;
    private static boolean CAMERA_ENABLED = false;
    public static final boolean NEW_ROBOT = true;
    private DcMotorEx flywheel_left = null;
    private DcMotorEx flywheel_right = null;

    @Override
    public void runOpMode() {/* Initialize shooter, drive, aprilTag, intake, and  distance sensor only if their respective enabled booleans are true */
        Shooter shooter = new Shooter();
        MecanumDrive drive = new MecanumDrive();
        AprilTag aprilTag = new AprilTag();
        Intake intake = new Intake();
        DistanceSensor distanceSensor = new DistanceSensor();


        if (SHOOT_ENABLED && !shooter.init(hardwareMap, gamepad2, telemetry, true, NEW_ROBOT)) {
            SHOOT_ENABLED = false;
        }
        if (DRIVE_ENABLED && !drive.init(hardwareMap, telemetry, gamepad1)) {
            DRIVE_ENABLED = false;
        }
        if (CAMERA_ENABLED && !aprilTag.init(hardwareMap, telemetry, gamepad1, drive)) {
            CAMERA_ENABLED = false;
        }
        if (INTAKE_ENABLED && !intake.init(hardwareMap, telemetry, gamepad2)) {
            INTAKE_ENABLED = false;
        }
        if (DISTANCE_ENABLED && !distanceSensor.init(hardwareMap, telemetry)) {
            DISTANCE_ENABLED = false;
        }

        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (SHOOT_ENABLED) {
                shooter.listen();
            }
            if (DRIVE_ENABLED) {
                drive.listen();
            }
            if (CAMERA_ENABLED) {
                aprilTag.listen();
            }
            if (INTAKE_ENABLED) {
                intake.listen();
            }
            if (DISTANCE_ENABLED) {
                distanceSensor.listen();
            }
            telemetry.update();
        }
    }
}
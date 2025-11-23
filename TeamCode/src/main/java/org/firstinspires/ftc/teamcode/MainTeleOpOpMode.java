package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.DistanceSensor;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp(name = "MainTeleOpOpModeCombinedAb11")
public class MainTeleOpOpMode extends LinearOpMode {

    private static boolean DRIVE_ENABLED = true;
    private static boolean SHOOT_ENABLED = true;
    private static boolean INTAKE_ENABLED = false;
    private static boolean DISTANCE_ENABLED = true;
    private static boolean CAMERA_ENABLED = false;
    public static final boolean NEW_ROBOT = true;

    @Override
    public void runOpMode() throws InterruptedException {
        /* Initialize shooter, drive, aprilTag, intake, and  distance sensor only if their respective enabled booleans are true */
        Shooter shooter = new Shooter();
        MecanumDrive drive = new MecanumDrive();
        AprilTag aprilTag = new AprilTag();
        Intake intake = new Intake();
        DistanceSensor distanceSensor = new DistanceSensor();

        telemetry.addLine("After constructor");
        if (SHOOT_ENABLED && !shooter.init(this, NEW_ROBOT)) {
            SHOOT_ENABLED = false;
        }
        if (DRIVE_ENABLED && !drive.init(this)) {
            DRIVE_ENABLED = false;
        }
        if (CAMERA_ENABLED && !aprilTag.init(this, drive)) {
            CAMERA_ENABLED = false;
        }
        if (INTAKE_ENABLED && !intake.init(this)) {
            INTAKE_ENABLED = false;
        }
        if (DISTANCE_ENABLED && !distanceSensor.init(this)) {
            DISTANCE_ENABLED = false;
        }
        telemetry.addLine("After init");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("Before shoot");
            telemetry.addData("Shoot enabled", SHOOT_ENABLED);
            if (SHOOT_ENABLED) {
                shooter.listen();
            }
            telemetry.addLine("After shoot");
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
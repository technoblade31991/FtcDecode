package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.DistanceSensor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp(name = "MainTeleOpOpModeCombinedAb11")
public class MainTeleOpOpMode extends LinearOpMode {

    public static final double INTAKE_MOTOR_POWER = 1.0;
    private static final boolean DRIVE_ENABLED = true;
    private static boolean SHOOT_ENABLED = true;
    private static boolean INTAKE_ENABLED = true;
    private static boolean DISTANCE_ENABLED = true;
    private static boolean CAMERA_ENABLED = false;
    private DcMotorEx intake_motor = null;
    private DcMotorEx flywheel_left = null;
    private DcMotorEx flywheel_right = null;

    @Override
    public void runOpMode() {
        // Initialize hardware

        // Initialize aprilTag
        AprilTag aprilTag = new AprilTag();

        /* Initialize distance sensor only if DISTANCE_ENABLED is true */
        DistanceSensor distanceSensor = new DistanceSensor();
        if (DISTANCE_ENABLED && !distanceSensor.init(hardwareMap, telemetry)) {
            DISTANCE_ENABLED = false;
        }

        // Initialize intake motor
        try {
            intake_motor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        } catch (Exception e) {
            telemetry.addData("ERROR", "intake_motor not found");
            telemetry.update();
            INTAKE_ENABLED = false;
        }

        if (SHOOT_ENABLED) {

            // Initialize flywheel motor
            try {
                flywheel_left = hardwareMap.get(DcMotorEx.class, "flywheel_left");
            } catch (Exception e) {
                telemetry.addData("ERROR", "flywheel_left motor not found");
                telemetry.update();
                // TODO: Should we still try to launch if only one flywheel is present?
                SHOOT_ENABLED = false;
            }

            try {
                flywheel_right = hardwareMap.get(DcMotorEx.class, "flywheel_right");
                flywheel_right.setDirection(DcMotorSimple.Direction.REVERSE);
            } catch (Exception e) {
                telemetry.addData("ERROR", "flywheel_right not found");
                telemetry.update();
                // TODO: Should we still try to launch if only one flywheel is present?
                SHOOT_ENABLED = false;

            }

        }
        /* Initialize AprilTag camera only if CAMERA_ENABLED is true */
        if (CAMERA_ENABLED && !aprilTag.init(hardwareMap)) {
            telemetry.addData("AprilTag", "Camera initialization failed!");
            telemetry.update();
            CAMERA_ENABLED = false;
        }
        flywheel_left.setPower(0.8);
        flywheel_right.setPower(0.8);
        intake_motor.setPower(0);

        MecanumDrive drive;
        // Initialize mecanum drive
        if (DRIVE_ENABLED) {
            drive = new MecanumDrive();
            drive.init(hardwareMap, telemetry);
        }
        Shooter shooter = null;
        if (SHOOT_ENABLED) {
            shooter = new Shooter();
            shooter.init(hardwareMap, gamepad2, telemetry);
        }

        waitForStart();
        while (opModeIsActive()) {
            if (CAMERA_ENABLED) {
                aprilTag.listen(telemetry, gamepad1, drive);
            }
            if (SHOOT_ENABLED) {
                assert shooter != null;
                shooter.listen(false);
            }
            if (DRIVE_ENABLED) {
                drive.listen(gamepad1);
            }
            if (DISTANCE_ENABLED) {
                distanceSensor.listen(telemetry);
            }
            telemetry.update();
        }
    }
}
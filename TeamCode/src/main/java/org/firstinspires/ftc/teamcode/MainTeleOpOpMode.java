package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp(name = "MainTeleOpOpModeCombinedAb01")
public class MainTeleOpOpMode extends LinearOpMode {

    double forward, strafe, rotate;

    private static final boolean DRIVE_ENABLED = true;
    private static final boolean SHOOT_ENABLED = false;
    private static boolean CAMERA_ENABLED = true;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        // Initialize hardware

        // Initialize aprilTag
        AprilTag aprilTag = new AprilTag();
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_Sensor");

        if (CAMERA_ENABLED) {
            if (!aprilTag.init(hardwareMap)) {
                telemetry.addData("AprilTag", "Camera initialization failed!");
                telemetry.update();
                aprilTag = null;
                CAMERA_ENABLED = false;
            }
        } else {
            aprilTag = null;
        }



        MecanumDrive drive;
        // Initialize mecanum drive
        if (DRIVE_ENABLED) {
            drive = new MecanumDrive();
            drive.init(hardwareMap);
        } else {
            drive = null;
        }
        Shooter shooter;
        if (SHOOT_ENABLED) {
            shooter = new Shooter();
            shooter.init(hardwareMap, gamepad2, telemetry);
        } else {
            shooter = null;
        }

        waitForStart();
        while (opModeIsActive()) {
            if (CAMERA_ENABLED) {
                aprilTag.listen(telemetry, gamepad1, drive);
            }
            if (SHOOT_ENABLED) {
                shooter.listen(false);
            }
            /* Mecanum drive control
             * Left stick Y axis = forward/backward
             * Left stick X axis = strafe left/right
             * Right stick X axis = rotate clockwise/counterclockwise
             * Drive relative to the field (not the robot) when right trigger is pressed
             */
            if (DRIVE_ENABLED) {
                forward = gamepad1.right_stick_y;
                // Strafe is reversed due to weird issues
                strafe = -gamepad1.right_stick_x;
                rotate = gamepad1.left_stick_x;
                drive.driveRelativeRobot(forward, strafe, rotate);

                double distance = distanceSensor.getDistance(DistanceUnit.INCH);

                // 2. Check the distance range using the logical AND operator (&&)
                if (distance > 24 && distance < 28) {
                    // Rumbles the controller for 5000ms (5 seconds)
                    // Note: Consider a shorter rumble or a pattern for better feedback
                    // gamepad1.rumble(1.0, 1.0, 3000);
                    telemetry.addData("Status", "TARGET IN RANGE");
                } else {
                    telemetry.addData("Status", "Keep driving...");
                }

                telemetry.addData("Distance (in)", distance);
                telemetry.update();
            }

            telemetry.update();
        }
    }
}
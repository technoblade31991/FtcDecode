package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp(name = "MainTeleOpOpModeCombinedAb01")
public class MainTeleOpOpMode extends LinearOpMode {

    double forward, strafe, rotate;
    private static final boolean DRIVE_ENABLED = true;



    @Override
    public void runOpMode() {
        // Initialize hardware
        AprilTag aprilTag = new AprilTag();
        aprilTag.init(hardwareMap);

        MecanumDrive drive;
        // Initialize mecanum drive
        if (DRIVE_ENABLED) {
            drive = new MecanumDrive();
            drive.init(hardwareMap);
        } else {
            drive = null;
        }
        Shooter shooter = new Shooter();
        shooter.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            aprilTag.addTelemetry();
            shooter.listen();
            /* Mecanum drive control
             * Left stick Y axis = forward/backward
             * Left stick X axis = strafe left/right
             * Right stick X axis = rotate clockwise/counterclockwise
             * Drive relative to the field (not the robot) when right trigger is pressed
             */
            if (DRIVE_ENABLED) {

                forward = gamepad1.right_stick_y;
                strafe = gamepad1.right_stick_x;
                rotate = gamepad1.left_stick_x;
                drive.driveRelativeRobot(forward, strafe, rotate);
            }

            telemetry.update();
        }
    }
}
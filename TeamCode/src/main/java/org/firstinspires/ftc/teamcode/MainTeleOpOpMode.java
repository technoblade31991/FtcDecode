package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp(name = "MainTeleOpOpModeCombinedAb02")
public class MainTeleOpOpMode extends LinearOpMode {

    double forward, strafe, rotate, maxSpeed;

    private static final boolean DRIVE_ENABLED = true;
    private static final boolean SHOOT_ENABLED = true;
    @Override
    public void runOpMode() {
        // Initialize hardware

        // Initialize aprilTag
        AprilTag aprilTag = new AprilTag();
        aprilTag.init(hardwareMap);

        MecanumDrive drive;
        // Initialize mecanum drive
        if (DRIVE_ENABLED) {
            drive = new MecanumDrive();
            drive.init(hardwareMap, telemetry);
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

            aprilTag.listen(telemetry, gamepad1, drive);
            if (SHOOT_ENABLED) {
                shooter.listen(false);
            }
            /* Mecanum drive control
             * Left stick Y axis = forward/backward
             * Left stick X axis = strafe left/right
             * Right stick X axis = rotate clockwise/counterclockwise
             * Drive relative to the field (not the robot) when right trigger is pressed
             * If button is pressed And DRIVE_ENABLED
            new mode call the driveRelativeRobot with maxSpeed = 0.5
            else call regular mode
             */

            if (gamepad1.left_trigger > 0.5 && DRIVE_ENABLED) {
                forward = gamepad1.right_stick_y;
                strafe = -gamepad1.right_stick_x;
                rotate = gamepad1.left_stick_x;
                drive.driveRelativeRobot(forward, strafe, rotate, 0.25);
            }
            else if (DRIVE_ENABLED) {
                    forward = gamepad1.right_stick_y;
                    // Strafe is reversed due to weird issues
                    strafe = -gamepad1.right_stick_x;
                    rotate = gamepad1.left_stick_x;
                    drive.driveRelativeRobot(forward, strafe, rotate, 1);
            }

            telemetry.update();
        }
    }
}
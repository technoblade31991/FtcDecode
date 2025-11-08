package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Disabled
@TeleOp(name = "Anya_MecanumRobotOrientatedOpModeAA")
public class MecanumRobotOrientatedOpMode extends OpMode {
    double forward, strafe, rotate;
    MecanumDrive drive = new MecanumDrive();
    @Override
    public void init() {
        drive.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        forward = gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        rotate = gamepad1.left_stick_x;

        drive.driveRelativeRobot(forward, strafe, rotate, 1);
    }
}
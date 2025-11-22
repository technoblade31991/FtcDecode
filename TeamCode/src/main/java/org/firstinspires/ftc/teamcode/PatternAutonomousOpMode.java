package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
@Disabled
@Autonomous(name="Pattern Autonomous Op Mode", group="Linear OpMode")
public class PatternAutonomousOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive();
        drive.init(hardwareMap, telemetry, gamepad1);
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
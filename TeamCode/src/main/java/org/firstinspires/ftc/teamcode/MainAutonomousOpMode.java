package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "MainAutonomousOpMode")
public class MainAutonomousOpMode extends LinearOpMode {


    private static final long DRIVE_START_FORWARD_MS = 10_000;


    @Override
    public void runOpMode(){
        // Initialize hardware

        // Initialize mecanum drive
        MecanumDrive drive = new MecanumDrive();
        drive.init(hardwareMap);

        // Drive forward for DRIVE_START_FORWARD_MS

        drive.driveRelativeField(1, 0, 0);
        sleep(DRIVE_START_FORWARD_MS);
        drive.driveRelativeField(0, 0, 0);
    }
}
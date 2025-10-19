package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTag;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@Autonomous(name = "MainAutonomousOpMode")
public class MainAutonomousOpMode extends LinearOpMode {


    private static final double DRIVE_START_FORWARD_MS = 10_000;


    @Override
    public void runOpMode(){
        // Initialize hardware

        // Initialize apriltag
        AprilTag aprilTag = new AprilTag();
        aprilTag.init(hardwareMap);

        // Initialize mecanum drive
        MecanumDrive drive = new MecanumDrive();
        drive.init(hardwareMap);

        double angleToTurn = 0;
        waitForStart();
        while (opModeIsActive()) {
            aprilTag.addTelemetry(telemetry);
            angleToTurn = -aprilTag.pose.bearing;
            drive.driveRelativeRobot(angleToTurn, 0, 0);
        }
    }
}
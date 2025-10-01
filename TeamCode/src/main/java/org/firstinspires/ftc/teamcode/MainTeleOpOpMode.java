package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "ContinuousLauncherAutonomousOpMode")
public class ContinuousLauncherAutonomousOpMode extends LinearOpMode {
    private CRServo left_feeder;
    private CRServo right_feeder;
    private DcMotor launcher;
    @Override
    public void runOpMode() {
        left_feeder = hardwareMap.crservo.get("left_feeder");
        right_feeder = hardwareMap.crservo.get("right_feeder");
        launcher = hardwareMap.dcMotor.get("launcher");
        left_feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            launcher.setPower(0.6);
            sleep(3000);
            left_feeder.setPower(1);
            right_feeder.setPower(1);
        }
    }
}
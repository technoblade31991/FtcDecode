package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name = "MainTeleOpOpModeCombinedAb01")
public class MainTeleOpOpMode extends LinearOpMode {
    private static final double LAUNCH_LAUNCHER_POWER = 0.6;
    private static final long LAUNCH_SLEEP_MS = 3000;
    private static final double LAUNCH_LEFT_FEEDER_POWER = 1;
    private static final double LAUNCH_RIGHT_FEEDER_POWER = 1;
    private static final double STOP_LAUNCHER_POWER = 0;
    private static final double STOP_LEFT_FEEDER_POWER = 0;
    private static final double STOP_RIGHT_FEEDER_POWER = 0;
    double forward, strafe, rotate;

    MecanumDrive drive;

    @Override
    public void runOpMode() {
        CRServo left_feeder = hardwareMap.crservo.get("left_feeder");
        CRServo right_feeder = hardwareMap.crservo.get("right_feeder");
        DcMotor launcher = null;

        drive  = new MecanumDrive();

        drive.init(hardwareMap);



        try {
            launcher = hardwareMap.dcMotor.get("launcher");
        } catch (Exception e) {
            telemetry.addData("ERROR", "LAUNCHER not found");
        }
        left_feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                if (launcher != null){
                    launcher.setPower(LAUNCH_LAUNCHER_POWER);
                }
                sleep(LAUNCH_SLEEP_MS);
                left_feeder.setPower(LAUNCH_LEFT_FEEDER_POWER);
                right_feeder.setPower(LAUNCH_RIGHT_FEEDER_POWER);
            } else if (gamepad1.left_bumper) {
                break;
            } else {
                if (launcher != null) {
                    launcher.setPower(STOP_LAUNCHER_POWER);
                }
                    left_feeder.setPower(STOP_LEFT_FEEDER_POWER);
                right_feeder.setPower(STOP_RIGHT_FEEDER_POWER);
            }
            forward = gamepad1.right_stick_y;
            strafe = gamepad1.right_stick_x;
            rotate = gamepad1.left_stick_x;
            drive.driveRelativeRobot(forward, strafe, rotate);
        }
    }
}
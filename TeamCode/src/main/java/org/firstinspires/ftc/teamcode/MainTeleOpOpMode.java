package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MainTeleOpOpMode")
public class MainTeleOpOpMode extends LinearOpMode {
    private static final double LAUNCH_LAUNCHER_POWER = 0.6;
    private static final long LAUNCH_SLEEP_MS = 3000;
    private static final double LAUNCH_LEFT_FEEDER_POWER = 1;
    private static final double LAUNCH_RIGHT_FEEDER_POWER = 1;
    private static final double STOP_LAUNCHER_POWER = 0;
    private static final double STOP_LEFT_FEEDER_POWER = 0;
    private static final double STOP_RIGHT_FEEDER_POWER = 0;
    double forward, strafe, rotate;


    @Override
    public void runOpMode() {
        DCMotor left_feeder = null;
        DCMotor right_feeder = null;
        DcMotor launcher = null;
        try {
            left_feeder = hardwareMap.crservo.get("left_feeder");
        } catch (Exception e) {
            telemetry.addData("ERROR", "LEFT FEEDER not found");
        }
        try {
            right_feeder = hardwareMap.crservo.get("right_feeder");
        } catch (Exception e) {
            telemetry.addData("ERROR", "RIGHT FEEDER not found");
        }
        try {
            launcher = hardwareMap.dcMotor.get("launcher");
        } catch (Exception e) {
            telemetry.addData("ERROR", "LAUNCHER not found");
        }
        // Set left feeder direction to REVERSE to make launching more intuitive by removing negative values
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
        }
    }
}

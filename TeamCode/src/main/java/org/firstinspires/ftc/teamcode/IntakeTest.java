package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "MainTeleOpOpModeIntakeAbhirama5")
public class IntakeTest  extends LinearOpMode {
    private static final double LAUNCH_LAUNCHER_POWER = 0.8;    
    private static final double LAUNCH_FLYWHEEL_POWER = 0.7;
    private static final double STOP_LAUNCHER_POWER = 0;
    private static final double STOP_FLYWHEEL_POWER = 0;

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        super.updateTelemetry(telemetry);
    }

    @Override
    public void runOpMode() {

        DcMotor launcher = null;
        DcMotor flywheel = null;
        DcMotor flywheel2 = null;


        try {
            launcher = hardwareMap.dcMotor.get("flywheel_right");
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        } catch (Exception e) {
            telemetry.addData("ERROR", "LAUNCHER not found");
        }

        try {
            flywheel = hardwareMap.dcMotor.get("flywheel_left");
        } catch (Exception e) {
            telemetry.addData("ERROR", "LAUNCHER not found");
        }

        try {
            flywheel2 = hardwareMap.dcMotor.get("intake_motor");

        } catch (Exception e) {
            telemetry.addData("ERROR", "LAUNCHER not found");
        }

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                if (launcher != null) {
                    launcher.setPower(LAUNCH_LAUNCHER_POWER);
                }
            }
            else {
                launcher.setPower(STOP_LAUNCHER_POWER);
            }


            if (gamepad1.left_bumper) {

                if (flywheel != null) {
                    flywheel.setPower(LAUNCH_FLYWHEEL_POWER);
                    flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

                }
                if (flywheel2 != null) {
                    flywheel2.setPower(LAUNCH_FLYWHEEL_POWER);
                }

            }
            else {
                flywheel.setPower(STOP_LAUNCHER_POWER);
                flywheel2.setPower(STOP_LAUNCHER_POWER);
           }
            }
        }
    }
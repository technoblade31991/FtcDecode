package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    private DcMotorEx intake_motor;
    public static final double INTAKE_MOTOR_FULL_POWER = 1;
    public static final double INTAKE_MOTOR_STOP_POWER = 0;
    private OpMode opMode;

    public boolean init(OpMode opMode) {
        this.opMode = opMode;
        try {
            this.intake_motor = opMode.hardwareMap.get(DcMotorEx.class, "intake_motor");
        } catch (Exception e) {
            opMode.telemetry.addData("ERROR", "intake_motor not found");
            return false;
        }
        return true;
    }

    public void listen() {
        if (this.opMode.gamepad1.dpad_up) {
            this.opMode.telemetry.addData("Intake", "On");
            intake_motor.setPower(INTAKE_MOTOR_FULL_POWER);
        } else if (this.opMode.gamepad1.dpad_down) {
            this.opMode.telemetry.addData("Intake", "Off");
            intake_motor.setPower(INTAKE_MOTOR_STOP_POWER);
        }
    }
}

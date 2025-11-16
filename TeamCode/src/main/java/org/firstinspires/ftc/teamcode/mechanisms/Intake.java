package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotorEx intake_motor;
    public static final double INTAKE_MOTOR_FULL_POWER = 1;
    public static final double INTAKE_MOTOR_STOP_POWER = 0;
    private Gamepad gamepad2;
    private Telemetry telemetry;

    public boolean init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad2) {
        try {
            this.intake_motor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        } catch (Exception e) {
            telemetry.addData("ERROR", "intake_motor not found");
            return false;
        }
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        return true;
    }

    public void listen() {
        if (this.gamepad2.dpad_up) {
            this.telemetry.addData("Intake", "Starting");
            intake_motor.setPower(INTAKE_MOTOR_FULL_POWER);
            this.telemetry.addData("Intake", "Started");
        } else if (this.gamepad2.dpad_down) {
            intake_motor.setPower(INTAKE_MOTOR_STOP_POWER);
        }
    }
}

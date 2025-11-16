package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    public boolean init( HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            Object intake_motor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        } catch (Exception e) {
            telemetry.addData("ERROR", "intake_motor not found");
            return false;
        }
        return true;
    }
}

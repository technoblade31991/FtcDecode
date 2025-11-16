package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;
    private static final String deviceName = "distance_Sensor";
    /*
    Returns true if initialization was successful, else false.
    */

    public boolean init(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            distanceSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.DistanceSensor.class, deviceName);

        } catch (Exception e) {
            telemetry.addData("ERROR", "distance_Sensor not found");
            return false;
        }
        return true;
    }

    public void listen(Telemetry telemetry) {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);

        // 2. Check the distance range using the logical AND operator (&&)
        if (distance > 24 && distance < 28) {
            // Rumbles the controller for 5000ms (5 seconds)
            // Note: Consider a shorter rumble or a pattern for better feedback
            // gamepad1.rumble(1.0, 1.0, 3000);
            telemetry.addData("Status", "TARGET IN RANGE");
        } else {
            telemetry.addData("Status", "Keep driving...");
        }

        telemetry.addData("Distance (in)", distance);
    }
}

package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor {
    private static final double MIN_DISTANCE = 24; // Minimum distance in inches
    private static final double MAX_DISTANCE = 28; // Maximum distance in inches
    private com.qualcomm.robotcore.hardware.DistanceSensor distanceSensor;
    private static final String deviceName = "distance_Sensor";
    private Telemetry telemetry;
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
        this.telemetry = telemetry;
        return true;
    }

    public void listen() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);

        // 2. Check the distance range using the logical AND operator (&&)
        if (distance > MIN_DISTANCE && distance < MAX_DISTANCE) {
            // Rumbles the controller for 5000ms (5 seconds)
            // Note: Consider a shorter rumble or a pattern for better feedback
            // gamepad1.rumble(1.0, 1.0, 3000);
            this.telemetry.addData("Status", "TARGET IN RANGE");
        } else {
            this.telemetry.addData("Status", "Keep driving...");
        }

        this.telemetry.addData("Distance (in)", distance);
    }
}

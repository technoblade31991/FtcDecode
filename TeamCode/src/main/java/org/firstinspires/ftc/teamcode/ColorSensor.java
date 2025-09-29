package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Basic Color Sensor", group = "Tutorial")
public class ColorSensor extends LinearOpMode {

    // 1. Declare your ColorSensor variable
    private com.qualcomm.robotcore.hardware.ColorSensor colorSensor;

    @Override
    public void runOpMode() {

        // 2. Map the sensor to the name in your robot's configuration
        // Make sure the name "sensor_color" matches your config!
        try {
            colorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor.class, "colorsensor");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find color sensor 'colorsensor'. Check configuration.");
            telemetry.update();
            sleep(5000); // Give user time to read error
            requestOpModeStop(); // Stop the OpMode if sensor isn't found
        }

        // You can optionally turn the sensor's LED on or off
        // By default, the LED is on.
        // colorSensor.enableLed(false); // Turn LED off
        // colorSensor.enableLed(true);  // Turn LED on

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start");
        telemetry.addData(">", "Point the sensor at different colors");
        telemetry.update();

        // Wait for the driver to press the START button
        waitForStart();

        while (opModeIsActive()) {

            // 3. Read the color sensor values
            // These values are typically integers from 0 to 255 (or higher)
            int redValue = colorSensor.red();
            int greenValue = colorSensor.green();
            int blueValue = colorSensor.blue();
            int alphaValue = colorSensor.alpha(); // This is the brightness/light intensity

            // 4. Send the values to the Driver Station screen
            telemetry.addData("Red", redValue);
            telemetry.addData("Green", greenValue);
            telemetry.addData("Blue", blueValue);
            telemetry.addData("Alpha (Brightness)", alphaValue);

            // 5. Add simple logic to detect a color
            // This is a very basic example. For real-world use,
            // you'll want to test and tune these thresholds.
            if (redValue > blueValue && redValue > greenValue) {
                telemetry.addData("Color Detected", "RED");
            } else if (blueValue > redValue && blueValue > greenValue) {
                telemetry.addData("Color Detected", "BLUE");
            } else if (greenValue > redValue && greenValue > blueValue) {
                telemetry.addData("Color Detected", "GREEN");
            } else {
                telemetry.addData("Color Detected", "---");
            }

            telemetry.update();

            // Always include a small pause in your loop
            sleep(20);
        }
    }
}
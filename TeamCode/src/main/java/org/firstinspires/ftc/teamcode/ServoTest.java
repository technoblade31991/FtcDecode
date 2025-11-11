package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Basic Servo Control", group = "Tutorial")
public class ServoTest extends LinearOpMode {

    // 1. Declare your Servo variable
    private Servo clawServo;

    // You can define constants for your servo positions
    // These values are from 0.0 (0 degrees) to 1.0 (usually 180 or 270 degrees)
    // You will need to tune these values for your specific servo and mechanism!
    static final double SERVO_OPEN_POSITION = 0.8;
    static final double SERVO_CLOSED_POSITION = 0.2;

    @Override
    public void runOpMode() {

        // 2. Map the servo to the name in your robot's configuration
        // "hardwareMap" is how your code talks to the hardware on the Control Hub
        // Make sure the name "claw_servo" matches your config!
        try {
            clawServo = hardwareMap.get(Servo.class, "servo1");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find servo 'servo1'. Check configuration.");
            telemetry.update();
            sleep(5000); // Give user time to read error
            requestOpModeStop(); // Stop the OpMode if servo isn't found
        }


        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start");
        telemetry.addData(">", "Press 'A' for OPEN, 'B' for CLOSED");
        telemetry.update();

        // Wait for the driver to press the START button on the Driver Station
        waitForStart();

        // runUntilStop() will not be called in a LinearOpMode, so we use a loop
        while (opModeIsActive()) {

            // 3. Set the servo's position based on gamepad input
            // Gamepad buttons return 'true' when pressed
            if (gamepad1.a) {
                // Move to the OPEN position
                clawServo.setPosition(SERVO_OPEN_POSITION);
            } else if (gamepad1.b) {
                // Move to the CLOSED position
                clawServo.setPosition(SERVO_CLOSED_POSITION);
            }

            // 4. Add Telemetry to see what's happening
            // This sends data back to the Driver Station screen
            telemetry.addData("Servo Position", clawServo.getPosition());
            telemetry.addData("Press 'A' for OPEN", SERVO_OPEN_POSITION);
            telemetry.addData("Press 'B' for CLOSED", SERVO_CLOSED_POSITION);
            telemetry.update();

            // Always include a small pause in your loop
            // to allow the SDK to run other processes
            sleep(20);
        }
    }
}
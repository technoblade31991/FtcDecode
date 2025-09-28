package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Random;

@Autonomous(name = "Auto servo test")
public class ServoAutoTest extends LinearOpMode {

    Servo servoToTest;

    @Override
    public void runOpMode() {
        // IMPORTANT: Change "your_servo_name_here" to the name you
        // gave the servo in your robot's configuration file.
        servoToTest = hardwareMap.get(Servo.class, "servo1");

        telemetry.addLine("Press PLAY to start the Servo ID Test.");
        telemetry.addLine("The servo will be commanded to position 0.8.");
        telemetry.addLine("---------------------------------------------");
        telemetry.addLine("IF IT MOVES AND STOPS: It is a STANDARD SERVO.");
        telemetry.addLine("IF IT SPINS AND KEEPS SPINNING: It is a CONTINUOUS SERVO.");
        telemetry.update();
        double position = new Random().nextDouble();
        waitForStart();

        while (opModeIsActive()) {
            // Send a command to move to a position other than the center.
            // A standard servo will go to this angle and stop.
            // A continuous servo will interpret this as "move at a constant speed."
            servoToTest.setPosition(position);

            // Keep the telemetry updated so you can see the instructions on the Driver Hub
            telemetry.addLine("Servo commanded to position 0.8.");
            telemetry.addLine("---------------------------------------------");
            telemetry.addLine("IF IT MOVES AND STOPS: It is a STANDARD SERVO.");
            telemetry.addLine("IF IT SPINS AND KEEPS SPINNING: It is a CONTINUOUS SERVO.");
            telemetry.update();
        }
    }
}
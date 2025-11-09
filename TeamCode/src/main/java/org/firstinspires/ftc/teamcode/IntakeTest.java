package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="FlywheelLaunchTest", group="Test")
public class IntakeTest extends LinearOpMode {
    private DcMotor intakeLeft = null;
    private DcMotor intakeRight = null;

    @Override
    public void runOpMode() {
        // Map hardware from configuration
        intakeLeft = hardwareMap.get(DcMotor.class, "intake_left");
        intakeRight = hardwareMap.get(DcMotor.class, "intake_right");

        // Optional: reverse one motor if needed for correct direction
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();

        // Wait for play button
        waitForStart();

        if (opModeIsActive()) {
            // Run both intake motors at full speed
            intakeLeft.setPower(1.0);
            intakeRight.setPower(1.0);

            telemetry.addLine("Intake Motors running at full speed");
            telemetry.update();

            // Keep running until stop is pressed
            while (opModeIsActive()) {
                // Do stuff
                idle();
            }

            // Stop motors when opmode ends
            intakeLeft.setPower(0);
            intakeRight.setPower(0);
        }
    }
}

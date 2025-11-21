package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    private DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    private IMU  imu;
    private Gamepad gamepad1;
    private Telemetry telemetry;

    public boolean init(HardwareMap hwMap, Telemetry telemetry, Gamepad gamepad1) {
        try {
            frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        } catch (Exception e) {
            telemetry.addData("ERROR", "frontLeftMotor not initialized");
        }
        try {
            frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        } catch (Exception e) {
            telemetry.addData("ERROR", "frontRightMotor not initialized");
        }
        try {
            backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        } catch (Exception e) {
            telemetry.addData("ERROR", "backLeftMotor not initialized");
        }
        try {
            backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");
        } catch (Exception e) {
            telemetry.addData("ERROR", "backRightMotor not initialized");
        }
        this.telemetry = telemetry;

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu =    hwMap.get(IMU.class, "imu");

        // Square orientation right angle control hub needed
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        );

        imu.initialize(new IMU.Parameters(revOrientation));
        this.gamepad1 = gamepad1;
        return true;
    }

    public void listen() {
        /* Mecanum drive control
             * Left stick Y axis = forward/backward
             * Left stick X axis = strafe left/right
             * Right stick X axis = rotate clockwise/counterclockwise
             * Drive relative to the field (not the robot) when right trigger is pressed
             * If button is pressed And DRIVE_ENABLED
            new mode call the driveRelativeRobot with maxSpeed = 0.5
            else call regular mode
             */
        /* Park assist mode when left trigger is pressed */
        double maxSpeed = 1;
        if (this.gamepad1.left_trigger > 0.5) {
            maxSpeed = 0.25;
            this.telemetry.addData("Drive Mode", "Park Assist");
        } else {
            this.telemetry.addData("Drive Mode", "Normal");
        }

        double forward = this.gamepad1.right_stick_y;
        // Strafe is reversed due to weird issues
        double strafe = -this.gamepad1.right_stick_x;
        double rotate = this.gamepad1.left_stick_x;
        this.driveRelativeRobot(forward, strafe, rotate, maxSpeed);

    }

    public void driveRelativeRobot(double forward, double strafe, double rotate, double maxSpeed) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));

    }


    public void driveRelativeField(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);
        theta = AngleUnit.normalizeRadians(theta - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.driveRelativeRobot(newForward, newStrafe, rotate, 1);
    }
}

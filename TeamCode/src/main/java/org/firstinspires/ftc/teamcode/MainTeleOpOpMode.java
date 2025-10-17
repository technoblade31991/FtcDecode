package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp(name = "MainTeleOpOpModeCombinedAb01")
public class MainTeleOpOpMode extends LinearOpMode {
    enum State {
        OFF,
        STARTING,
        FEED,
        LAUNCHING,
        STOPPING,

    }

    private State state = State.OFF;
    private final ElapsedTime timer = new ElapsedTime();

    private static final double LAUNCH_LAUNCHER_POWER = 0.4;
    private static final double LAUNCH_LAUNCHER_FULL_SPEED_MS = 3000;
    private static final double LAUNCH_LEFT_FEEDER_POWER = 1;
    private static final double LAUNCH_RIGHT_FEEDER_POWER = 1;
    private static final double LAUNCH_STOP_LEFT_FEEDER_POWER = 0;
    private static final double LAUNCH_STOP_RIGHT_FEEDER_POWER = 0;
    private static final double LAUNCH_FEED_MS = 1_200;
    private static final double LAUNCH_LAUNCHER_COMPLETE_MS = 3000;

    double forward, strafe, rotate;

    MecanumDrive drive;
    private CRServo left_feeder = null;
    private CRServo right_feeder = null;
    private DcMotor launcher = null;

    @Override
    public void runOpMode() {
        // Initialize hardware

        // Initialize mecanum drive
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        // Initialize left feeder servo and set to reverse direction
        try {
            left_feeder = hardwareMap.crservo.get("left_feeder");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Left feeder not found");
            telemetry.update();
        }
        assert left_feeder != null;
        left_feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize right feeder servo
        try {
            right_feeder = hardwareMap.crservo.get("right_feeder");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Right feeder not found");
            telemetry.update();
        }
        assert right_feeder != null;

        // Initialize launcher motor
        try {
            launcher = hardwareMap.dcMotor.get("launcher");
        } catch (Exception e) {
            telemetry.addData("ERROR", "LAUNCHER not found");
            telemetry.update();
        }
        assert launcher != null;

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("state", state);
            telemetry.update();
            switch (state) {
                case OFF:
                    if (gamepad2.right_bumper) {
                        /*
                         * Right bumper was pressed.
                         * Launching ball.
                         * Set state to STARTING, reset the timer and
                         * start the launcher motor.
                         */
                        state = State.STARTING;

                        timer.reset();
                        launcher.setPower(LAUNCH_LAUNCHER_POWER);

                    }
                    break;
                case STARTING:
                    if (timer.milliseconds() > LAUNCH_LAUNCHER_FULL_SPEED_MS) {
                        /* Once launcher motor is at full speed,
                         * set the state to FEED, reset the timer and
                         * start the launcher servos.
                         */
                        state = State.FEED;

                        timer.reset();
                        left_feeder.setPower(LAUNCH_LEFT_FEEDER_POWER);
                        right_feeder.setPower(LAUNCH_RIGHT_FEEDER_POWER);
                    }
                    break;
                case FEED:
                    if (timer.milliseconds() > LAUNCH_FEED_MS) {
                        /* Once feed time is complete,transition to LAUNCHING state,
                         * start the timer and
                         * stop the servos so the second ball does not feed in.
                         */
                        state = State.LAUNCHING;
                        timer.reset();
                        left_feeder.setPower(LAUNCH_STOP_LEFT_FEEDER_POWER);
                        right_feeder.setPower(LAUNCH_STOP_RIGHT_FEEDER_POWER);
                    }
                    break;
                case LAUNCHING:
                    if (timer.seconds() > LAUNCH_LAUNCHER_COMPLETE_MS) {
                        /* Once launch time is complete,transition to STOPPING state.
                         * reset the timer and
                         * turn off the launcher so the battery does not drain
                         */
                        state = State.STOPPING;
                        timer.reset();
                        launcher.setPower(LAUNCH_LAUNCHER_POWER);
                    }
                    break;
                case STOPPING:
                    /* Stop all motors and servos and transition to OFF state */
                    left_feeder.setPower(LAUNCH_STOP_LEFT_FEEDER_POWER);
                    right_feeder.setPower(LAUNCH_STOP_RIGHT_FEEDER_POWER);
                    launcher.setPower(LAUNCH_LAUNCHER_POWER);
                    state = State.OFF;
                    break;
            }
            /* At any point, if the left bumper is pressed,
             * transition to STOPPING state to stop the launch sequence.
             */
            if (gamepad2.left_bumper) {
                state = State.STOPPING;
                break;
            }
            /* Mecanum drive control
             * Left stick Y axis = forward/backward
             * Left stick X axis = strafe left/right
             * Right stick X axis = rotate clockwise/counterclockwise
             * Drive relative to the field (not the robot) when right trigger is pressed
             */
            forward = gamepad1.right_stick_y;
            strafe = gamepad1.right_stick_x;
            rotate = gamepad1.left_stick_x;
            drive.driveRelativeRobot(forward, strafe, rotate);
        }
    }
}
package org.firstinspires.ftc.teamcode.mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {

    private enum State {
        OFF,
        STARTING,
        FEED,
        LAUNCHING,
        STOPPING,

    }

    private State state = State.OFF;
    @Configurable
    public static class ShooterConfig {

        public static double LAUNCH_LAUNCHER_POWER = 0.6;
        public static double LAUNCH_LAUNCHER_FULL_SPEED_MS = 3_000;
        public static double LAUNCH_LEFT_FEEDER_POWER = 1;
        public static double LAUNCH_RIGHT_FEEDER_POWER = 1;
        public static double LAUNCH_STOP_LEFT_FEEDER_POWER = 0;
        public static double LAUNCH_STOP_RIGHT_FEEDER_POWER = 0;
        public static double LAUNCH_STOP_LAUNCHER_POWER = 0;
        public static double LAUNCH_FEED_MS_RESET = 1_200;
        public static double LAUNCH_FEED_MS_NO_RESET = 1_200;
        public static double LAUNCH_LAUNCHER_COMPLETE_MS = 200;
    }
    private static boolean reset = true;
    private double launchFeedMs;
    private CRServo left_feeder = null;
    private CRServo right_feeder = null;
    private DcMotor launcher = null;
    private Gamepad gamepad2;
    private Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();
    public void init(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry) {
        // Add gamepad2 to self
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
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
        // Initialize launchFeedMs
        this.launchFeedMs = 0;
    }

    public void listen() {

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
                /* When ball is inserted, reset is true
                 * After feeding a reset launch, reset is set to false
                 * When second and third balls are launching, reset is false
                 * When reset is true, LAUNCH_FEED_MS_RESET is used
                 * When reset is false, LAUNCH_FEED_MS_NO_RESET is used
                 * This is due to the fact that the first time a ball is launched,
                 * the ball needs less time because it has not been fed.
                 * However, due to inconsistencies, when the second and third ball
                 * are launched, they are slightly fed.
                 * This is why we give the second and third balls less time
                 * So we set LAUNCH_FEED_MS based off of the boolean reset.
                 */
                if (reset) {
                    reset = false;
                    launchFeedMs = LAUNCH_FEED_MS_RESET;
                } else {
                    launchFeedMs = LAUNCH_FEED_MS_NO_RESET;
                }
                if (timer.milliseconds() > launchFeedMs) {
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
                if (timer.milliseconds() > LAUNCH_LAUNCHER_COMPLETE_MS) {
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
                launcher.setPower(LAUNCH_STOP_LAUNCHER_POWER);
                state = State.OFF;
                break;
        }
        /* At any point, if the left bumper is pressed,
         * transition to STOPPING state to stop the launch sequence.
         */
        if (gamepad2.left_bumper) {
            state = State.STOPPING;
            return;
        }
        this.telemetry.addData("state", state);
    }
}
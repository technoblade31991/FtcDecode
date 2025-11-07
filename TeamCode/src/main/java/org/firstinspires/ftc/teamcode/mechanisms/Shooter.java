package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.messages.TankLocalizerInputsMessage;

public class Shooter {

    private enum State {
        OFF,
        STARTING,
        FEED,
        LAUNCHING,
        STOPPING
    }

    /* Current state of the shooter mechanism */
    private State state = State.OFF;
    /* Power constants for the launcher and servos. */
    public static final double LAUNCH_STOP_LAUNCHER_POWER = 0; /* Launcher off power */
    public static final double LAUNCH_LAUNCHER_POWER = 0.6; /* Launcher running power */
    public static final double LAUNCHER_TARGET_VELOCITY = 1390; /* Target velocity for launcher */
    public static final double LAUNCH_STOP_LEFT_FEEDER_POWER = 0; /* Left feeder off power */
    public static final double LAUNCH_STOP_RIGHT_FEEDER_POWER = 0; /* Right feeder off power */
    public static final double LAUNCH_LEFT_FEEDER_POWER = 1; /* Left feeder running power */
    public static final double LAUNCH_RIGHT_FEEDER_POWER = 1; /* Right feeder running power */

    /* Time constants for the launcher and servos. */
    public static final double LAUNCH_LAUNCHER_FULL_SPEED_MS = 1_500; /* Time for launcher to reach full speed */
    public static final double LAUNCH_FEED_MS_RESET = 1_150; /* Time it takes for ball to pass through servos when reset is true */
    public static final double LAUNCH_FEED_MS_NO_RESET = 1_150; /* Time it takes for ball to pass through servos when reset is false.
                                                           * Note that currently this is equal to LAUNCH_FEED_MS_RESET.
                                                           */
    public static final double LAUNCH_LAUNCHER_COMPLETE_MS = 200; /* Time for ball to launch once it is past servos */

    private static final double LAUNCH_LAUNCHER_ENFORCE_MS = 300; /* Time to wait before re-feeding when enforcing continuous launch */
    private double launchFeedMs;
    private static boolean reset = true;
    private CRServo left_feeder = null;
    private CRServo right_feeder = null;

    private DcMotorEx launcher = null;
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
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        } catch (Exception e) {
            telemetry.addData("ERROR", "LAUNCHER not found");
            telemetry.update();
        }
        assert launcher != null;
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /* Use PIDF coefficients to control launcher velocity */
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(300,0,0,10));

        // Initialize launchFeedMs
        this.launchFeedMs = 0;
    }

    public boolean isOff() {
        /* Here we check for STOPPING not OFF, because when we have multiple launches,
         * we transition from STOPPING to FEED directly.
         */
        return state == State.STOPPING;
    }
    public boolean isFeeding(){
        return state == State.FEED;
    }
    public void listen(boolean enforce) {

        switch (state) {
            case OFF:
                if (gamepad2.right_bumper||enforce) {
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
                if ((launcher.getVelocity() >= LAUNCHER_TARGET_VELOCITY) || (timer.milliseconds() > LAUNCH_LAUNCHER_FULL_SPEED_MS)) {
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
                telemetry.addData("Launch Velocity",launcher.getVelocity());
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
                /* In case of continuous launch enforcement, we need to go back to FEED state */
                if (gamepad2.right_bumper || enforce) {
                    if ((launcher.getVelocity() >= LAUNCHER_TARGET_VELOCITY) ||(timer.milliseconds() > LAUNCH_LAUNCHER_ENFORCE_MS)) {
                        state = State.FEED;
                        left_feeder.setPower(LAUNCH_LEFT_FEEDER_POWER);
                        right_feeder.setPower(LAUNCH_RIGHT_FEEDER_POWER);

                        timer.reset();
                        break;
                    }
                } else {
                    /* Stop all motors and servos and transition to OFF state */
                    left_feeder.setPower(LAUNCH_STOP_LEFT_FEEDER_POWER);
                    right_feeder.setPower(LAUNCH_STOP_RIGHT_FEEDER_POWER);
                    launcher.setPower(LAUNCH_STOP_LAUNCHER_POWER);
                    state = State.OFF;
                }
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
    public void launch_n_balls(int n){

        /* Launch n balls in succession */
        for (int i = 0; i < n; i++) {
            telemetry.addData("Shooting", "Ball %d", i + 1);

            telemetry.update();
            listen(true);

            while(!isFeeding()){
                telemetry.addData("Shooting loop", "Ball %d", i + 1);
                listen(true);
                telemetry.update();
            }
            while (!isOff()) {
                telemetry.addData("Shooting loop", "Ball %d", i + 1);
                listen(true);
                telemetry.update();
            }
        }
    }
}
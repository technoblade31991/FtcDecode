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

public class Shooter {

    private boolean enforce;
    private boolean newRobot;
    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;

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
    public static final double LAUNCH_FEED_MS = 1_150; /* Time it takes for ball to pass through servos */
    public static final double LAUNCH_LAUNCHER_COMPLETE_MS = 200; /* Time for ball to launch once it is past servos */

    private static final double LAUNCH_LAUNCHER_ENFORCE_MS = 300; /* Time to wait before re-feeding when enforcing continuous launch */
    private CRServo left_feeder = null;
    private CRServo right_feeder = null;

    private DcMotorEx launcher;
    private Gamepad gamepad2;
    private Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    /*
     * Returns true if initialization was successful, else false.
     * newRobot indicates whether this is a new robot configuration, the one with two flywheels.
     */
    public boolean init(HardwareMap hardwareMap, Gamepad gamepad2, Telemetry telemetry, boolean teleOp, boolean newRobot) {
        // Add gamepad2 to self
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
        this.enforce = teleOp;
        this.newRobot = newRobot;
        // Initialize left feeder servo and set to reverse direction
        try {
            left_feeder = hardwareMap.crservo.get("left_feeder");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Left feeder not found");
            return false;
        }
        left_feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize right feeder servo
        try {
            right_feeder = hardwareMap.crservo.get("right_feeder");
        } catch (Exception e) {
            telemetry.addData("ERROR", "Right feeder not found");
            return false;
        }

        // Initialize launcher motor(s)
        if (newRobot) {
            if (!init_new_robot_launcher(hardwareMap)) {
                return false;
            }
        } else {
            if (!init_old_robot_launcher(hardwareMap)) {
                return false;
            }
        }
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /* Use PIDF coefficients to control launcher velocity */
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(300,0,0,10));
        return true;
    }

    private boolean init_old_robot_launcher(HardwareMap hardwareMap) {
        // TODO: Should we still try to launch if only one flywheel is present?
        try {
            this.leftLauncher = hardwareMap.get(DcMotorEx.class, "flywheel_left");
        } catch (Exception e) {
            telemetry.addData("ERROR", "flywheel_left not found");
            return false;
        }
        try {
            this.rightLauncher = hardwareMap.get(DcMotorEx.class, "flywheel_right");
            this.rightLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("ERROR", "flywheel_right not found");
            return false;
        }
        this.launcher = null;
        return true;
    }

    private boolean init_new_robot_launcher(HardwareMap hardwareMap) {
        try {
            this.launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        } catch (Exception e) {
            telemetry.addData("ERROR", "launcher not found");
            return false;
        }
        this.leftLauncher = null;
        this.rightLauncher = null;
        return true;
    }

    public boolean isOff() {
        /* Here we check for STOPPING not OFF, because when we have multiple launches,
         * we transition from STOPPING to FEED directly.
         */
        return state == State.STOPPING;
    }

    private void setPower(double power){
        if (this.newRobot) {
            launcher.setPower(power);
        } else {
            leftLauncher.setPower(power);
            rightLauncher.setPower(power);
        }
    }

    public boolean isFeeding(){
        return state == State.FEED;
    }
    public void listen() {
        switch (state) {
            case OFF:
                if (gamepad2.right_bumper||!this.enforce) {
                    /*
                     * Right bumper was pressed.
                     * Launching ball.
                     * Set state to STARTING, reset the timer and
                     * start the launcher motor.
                     */
                    state = State.STARTING;

                    timer.reset();
                    this.setPower(LAUNCH_LAUNCHER_POWER);

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
                if (timer.milliseconds() > LAUNCH_LAUNCHER_COMPLETE_MS) {
                    /* Once launch time is complete,transition to STOPPING state.
                     * reset the timer and
                     * turn off the launcher so the battery does not drain
                     */
                    state = State.STOPPING;
                    timer.reset();
                    this.setPower(LAUNCH_LAUNCHER_POWER);
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
                    this.setPower(LAUNCH_STOP_LAUNCHER_POWER);
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
            listen();

            while(!isFeeding()){
                telemetry.addData("Shooting loop", "Ball %d", i + 1);
                listen();
                telemetry.update();
            }
            while (!isOff()) {
                telemetry.addData("Shooting loop", "Ball %d", i + 1);
                listen();
                telemetry.update();
            }
        }
    }
}
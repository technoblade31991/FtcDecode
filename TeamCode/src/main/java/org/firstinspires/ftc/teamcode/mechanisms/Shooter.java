package org.firstinspires.ftc.teamcode.mechanisms;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Shooter {
    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;
    private OpMode opMode;

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
    public static double LAUNCH_LAUNCHER_POWER = 0.8; /* Launcher running power */
    public static double LAUNCHER_TARGET_VELOCITY = 5500; /* Target velocity for launcher */
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
    private final ElapsedTime timer = new ElapsedTime();
    private static final PIDFCoefficients PIDF_COEFFICIENTS = new PIDFCoefficients(300, 0, 0, 10);

    /*
     * Returns true if initialization was successful, else false.
     * newRobot indicates whether this is a new robot configuration, the one with two flywheels.
     */
    public boolean init(OpMode opMode, boolean NEW_ROBOT) {
        this.opMode = opMode;
        // Initialize left feeder servo and set to reverse direction
        try {
            left_feeder = opMode.hardwareMap.crservo.get("left_feeder");
        } catch (Exception e) {
            opMode.telemetry.addData("ERROR", "Left feeder not found");
        }
        left_feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize right feeder servo
        try {
            right_feeder = opMode.hardwareMap.crservo.get("right_feeder");
        } catch (Exception e) {
            opMode.telemetry.addData("ERROR", "Right feeder not found");
        }

        // Initialize launcher motor(s)
        if (this.NEW_ROBOT) {
            return init_new_robot_launcher(this.opMode.hardwareMap);
        } else {
            return init_old_robot_launcher(this.opMode.hardwareMap);
        }
    }

    private boolean init_new_robot_launcher(HardwareMap hardwareMap) {
        try {
            this.leftLauncher = opMode.hardwareMap.get(DcMotorEx.class, "flywheel_left");
            this.leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);

        } catch (Exception e) {
            this.opMode.telemetry.addData("ERROR", "flywheel_left not found");
            return false;
        }
        try {
            this.rightLauncher = opMode.hardwareMap.get(DcMotorEx.class, "flywheel_right");
        } catch (Exception e) {
            this.opMode.telemetry.addData("ERROR", "flywheel_right not found");
            return false;
        }
        this.leftLauncher.setZeroPowerBehavior(BRAKE);
        this.rightLauncher.setZeroPowerBehavior(BRAKE);
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /* Use PIDF coefficients to control launcher velocity */
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF_COEFFICIENTS);
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF_COEFFICIENTS);
        this.launcher = null;
        return true;
    }

    private boolean init_old_robot_launcher(HardwareMap hardwareMap) {
        try {
            this.launcher = opMode.hardwareMap.get(DcMotorEx.class, "launcher");
        } catch (Exception e) {
            this.opMode.telemetry.addData("ERROR", "launcher not found");
            return false;
        }
        this.launcher.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /* Use PIDF coefficients to control launcher velocity */
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF_COEFFICIENTS);
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

    private void setLauncherPower(double power){
        if (this.newRobot) {
            leftLauncher.setPower(power);
            rightLauncher.setPower(power);
        } else {
            launcher.setPower(power);
        }
    }

    private double getLauncherVelocity(){
        if (this.newRobot) {
            return (leftLauncher.getVelocity() + rightLauncher.getVelocity()) / 2.0;
        } else {
            return launcher.getVelocity();
        }
    }

    public boolean isFeeding(){
        return state == State.FEED;
    }
    public void listen()  {
        this.opMode.telemetry.addData("Velocity", LAUNCHER_TARGET_VELOCITY);
        if (this.opMode.gamepad2.dpadUpWasPressed()) {
            LAUNCHER_TARGET_VELOCITY += 100;
        } else if (this.opMode.gamepad2.dpadDownWasPressed()) {
            LAUNCHER_TARGET_VELOCITY -= 100;
        }

        this.opMode.telemetry.addData("Power", LAUNCH_LAUNCHER_POWER);
        if (this.opMode.gamepad2.dpadRightWasPressed()) {
            LAUNCH_LAUNCHER_POWER += 0.01;
        } else if (this.opMode.gamepad2.dpadLeftWasPressed()) {
            LAUNCH_LAUNCHER_POWER -= 0.01;
        }
        switch (state) {
            case OFF:
                if (this.opMode.gamepad2.right_bumper||this.autonomous) {
                    /*
                     * Right bumper was pressed.
                     * Launching ball.
                     * Set state to STARTING, reset the timer and
                     * start the launcher motor.
                     */
                    state = State.STARTING;

                    timer.reset();
                    this.setLauncherPower(LAUNCH_LAUNCHER_POWER);

                }
                break;
            case STARTING:
                if ((this.getLauncherVelocity() >= LAUNCHER_TARGET_VELOCITY) || (timer.milliseconds() > LAUNCH_LAUNCHER_FULL_SPEED_MS)) {
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
                this.opMode.telemetry.addData("Launch Velocity",this.getLauncherVelocity());
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
                    this.setLauncherPower(LAUNCH_LAUNCHER_POWER);
                }

                break;
            case STOPPING:
                /* In case of continuous launch enforcement, we need to go back to FEED state */
                if (this.opMode.gamepad2.right_bumper || autonomous) {
                    if ((this.getLauncherVelocity() >= LAUNCHER_TARGET_VELOCITY) ||(timer.milliseconds() > LAUNCH_LAUNCHER_ENFORCE_MS)) {
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
                    this.setLauncherPower(LAUNCH_STOP_LAUNCHER_POWER);
                    state = State.OFF;
                }
                break;
        }
        /* At any point, if the left bumper is pressed,
         * transition to STOPPING state to stop the launch sequence.
         */
        if (this.opMode.gamepad2.left_bumper) {
            state = State.STOPPING;
            return;
        }
        this.opMode.telemetry.addData("state", state);
    }
    public void launch_n_artifacts(int n)  {

        /* Launch n artifacts in succession */
        for (int i = 0; i < n; i++) {
            this.opMode.telemetry.addData("Shooting", "Ball %d", i + 1);

            this.opMode.telemetry.update();
            listen();

            while(!isFeeding()){
                this.opMode.telemetry.addData("Shooting loop", "Ball %d", i + 1);
                listen();
                this.opMode.telemetry.update();
            }
            while (!isOff()) {
                this.opMode.telemetry.addData("Shooting loop", "Ball %d", i + 1);
                listen();
                this.opMode.telemetry.update();
            }
        }
    }
}
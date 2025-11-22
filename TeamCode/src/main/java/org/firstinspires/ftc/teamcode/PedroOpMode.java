package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroOpMode", group = "Autonomous")
@Configurable // Panels
public class PedroOpMode extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private static boolean SHOOT_ENABLED = true;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        // Paths defined in the Paths class
        Paths paths = new Paths(follower); // Build paths

        // Initialize shooter mechanism
        Shooter shooter;
        if (SHOOT_ENABLED) {
            shooter = new Shooter();
            shooter.init(hardwareMap, gamepad2, telemetry, true, true);
        } else {
            shooter = null;
        }

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain GettoBalls1;
        public PathChain IntakeBalls1;
        public PathChain ShootBalls1;
        public PathChain GettoBalls2;
        public PathChain IntakeBalls2;
        public PathChain ShootBalls2;
        public PathChain GettoBalls3;
        public PathChain IntakeBalls3;
        public PathChain ShootBalls3;

        public Paths(Follower follower) {
            GettoBalls1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.000, 12.000), new Pose(40.685, 35.451))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                    .build();

            IntakeBalls1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.685, 35.451), new Pose(23.972, 35.620))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ShootBalls1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.972, 35.620), new Pose(60.098, 11.986))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();

            GettoBalls2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.098, 11.986), new Pose(40.853, 60.436))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                    .build();

            IntakeBalls2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.853, 60.436), new Pose(24.478, 60.436))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ShootBalls2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.478, 60.436), new Pose(59.761, 12.830))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();

            GettoBalls3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.761, 12.830), new Pose(40.853, 84.070))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                    .build();

            IntakeBalls3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.853, 84.070), new Pose(24.141, 83.564))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ShootBalls3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.141, 83.564), new Pose(60.277, 12.241))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();
        }
    }

    private enum AutonomousState {
        SHOOT_PRELOAD,
        GET_BALLS_1_PATH,
        INTAKE_BALLS_1_PATH,
        SHOOT_BALLS_1_PATH,
        SHOOT_BALLS_1,
        GET_BALLS_2_PATH,
        INTAKE_BALLS_2_PATH,
        SHOOT_BALLS_2_PATH,
        SHOOT_BALLS_2,
        GET_BALLS_3_PATH,
        INTAKE_BALLS_3_PATH,
        SHOOT_BALLS_3_PATH,
        SHOOT_BALLS_3
    }
    AutonomousState currentState = AutonomousState.SHOOT_PRELOAD;
    public int autonomousPathUpdate() {
        switch (currentState) {
            case SHOOT_PRELOAD:
                currentState = AutonomousState.GET_BALLS_1_PATH;
                break;

            case GET_BALLS_1_PATH:
                if (!follower.isPathActive()) {
                    follower.followPath(new Paths(follower).GettoBalls1);
                }
                if (follower.isPathCompleted()) {
                    currentState = AutonomousState.INTAKE_BALLS_1_PATH;
                }
                break;

            case INTAKE_BALLS_1_PATH:
                if (!follower.isPathActive()) {
                    follower.followPath(new Paths(follower).IntakeBalls1);
                }
                if (follower.isPathCompleted()) {
                    currentState = AutonomousState.SHOOT_BALLS_1_PATH;
                }
                break;

            case SHOOT_BALLS_1_PATH:
                if (!follower.isPathActive()) {
                    follower.followPath(new Paths(follower).ShootBalls1);
                }
                if (follower.isPathCompleted()) {
                    currentState = AutonomousState.SHOOT_BALLS_1;
                }
                break;

            case SHOOT_BALLS_1:
                // Implement shooting logic here
                currentState = AutonomousState.GET_BALLS_2_PATH;
                break;

            case GET_BALLS_2_PATH:
                if (!follower.isPathActive()) {
                    follower.followPath(new Paths(follower).GettoBalls2);
                }
                if (follower.isPathCompleted()) {
                    currentState = AutonomousState.INTAKE_BALLS_2_PATH;
                }
                break;

            case INTAKE_BALLS_2_PATH:
                if (!follower.isPathActive()) {
                    follower.followPath(new Paths(follower).IntakeBalls2);
                }
                if (follower.isPathCompleted()) {
                    currentState = AutonomousState.SHOOT_BALLS_2_PATH;
                }
                break;

            case SHOOT_BALLS_2_PATH:
                if (!follower.isPathActive()) {
                    follower.followPath(new Paths(follower).ShootBalls2);
                }
                if (follower.isPathCompleted()) {
                    currentState = AutonomousState.SHOOT_BALLS_2;
                }
                break;

            case SHOOT_BALLS_2:
                // Implement shooting logic here
                currentState = AutonomousState.GET_BALLS_3_PATH;
        }
        return pathState;
    }
}
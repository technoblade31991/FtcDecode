package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.MainTeleOpOpMode.NEW_ROBOT;

import static java.lang.Thread.sleep;

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
    private Shooter shooter;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        // Paths defined in the Paths class
        Paths paths = new Paths(follower); // Build paths

        // Initialize shooter mechanism
        this.shooter = new Shooter();
        if (SHOOT_ENABLED && !shooter.init(this)) {
            SHOOT_ENABLED = false;
        }
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        try {
            pathState = autonomousPathUpdate(); // Update autonomous state machine
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain GettoArtifacts1;
        public PathChain IntakeArtifacts1;
        public PathChain ShootArtifacts1;
        public PathChain GettoArtifacts2;
        public PathChain IntakeArtifacts2;
        public PathChain ShootArtifacts2;
        public PathChain GettoArtifacts3;
        public PathChain IntakeArtifacts3;
        public PathChain ShootArtifacts3;

        public Paths(Follower follower) {
            GettoArtifacts1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.000, 12.000), new Pose(40.685, 35.451))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                    .build();

            IntakeArtifacts1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.685, 35.451), new Pose(23.972, 35.620))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ShootArtifacts1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.972, 35.620), new Pose(60.098, 11.986))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();

            GettoArtifacts2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.098, 11.986), new Pose(40.853, 60.436))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                    .build();

            IntakeArtifacts2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.853, 60.436), new Pose(24.478, 60.436))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ShootArtifacts2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.478, 60.436), new Pose(59.761, 12.830))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();

            GettoArtifacts3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.761, 12.830), new Pose(40.853, 84.070))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                    .build();

            IntakeArtifacts3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.853, 84.070), new Pose(24.141, 83.564))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ShootArtifacts3 = follower
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
        GET_ARTIFACTS_1_PATH,
        INTAKE_ARTIFACTS_1_PATH,
        SHOOT_ARTIFACTS_1_PATH,
        SHOOT_ARTIFACTS_1,
        GET_ARTIFACTS_2_PATH,
        INTAKE_ARTIFACTS_2_PATH,
        SHOOT_ARTIFACTS_2_PATH,
        SHOOT_ARTIFACTS_2,
        GET_ARTIFACTS_3_PATH,
        INTAKE_ARTIFACTS_3_PATH,
        SHOOT_ARTIFACTS_3_PATH,
        SHOOT_ARTIFACTS_3
    }
    AutonomousState currentState = AutonomousState.SHOOT_PRELOAD;
    public int autonomousPathUpdate() throws InterruptedException {
        switch (currentState) {
            case SHOOT_PRELOAD:
                // In this state, we want to shoot the preloaded artifacts
                shooter.launch_n_artifacts(3);
                currentState = AutonomousState.GET_ARTIFACTS_1_PATH;
                break;

            case GET_ARTIFACTS_1_PATH:
                // Follow path to get to the position to intake artifacts 1
                follower.followPath(new Paths(follower).GettoArtifacts1);
                // Wait until the path is completed
                if (!follower.isBusy()) {
                    currentState = AutonomousState.INTAKE_ARTIFACTS_1_PATH;
                }
                break;

            case INTAKE_ARTIFACTS_1_PATH:
                // Follow path to intake artifacts 1
                follower.followPath(new Paths(follower).IntakeArtifacts1);
                // Wait until the path is completed
                if (!follower.isBusy()) {
                    currentState = AutonomousState.SHOOT_ARTIFACTS_1_PATH;
                }
                break;

            case SHOOT_ARTIFACTS_1_PATH:
                // Follow path to shoot artifacts 1
                follower.followPath(new Paths(follower).ShootArtifacts1);
                break;

            case SHOOT_ARTIFACTS_1:
                shooter.launch_n_artifacts(3);
                currentState = AutonomousState.GET_ARTIFACTS_2_PATH;
                break;
        return pathState;
    }
}
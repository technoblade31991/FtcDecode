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
@Configurable
public class PedroOpMode extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Shooter shooter;

    private Paths paths;

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

    private AutonomousState currentState = AutonomousState.SHOOT_PRELOAD;
    private boolean stateStarted = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        shooter = new Shooter();
        boolean shootEnabled = true;
        if (shootEnabled && !shooter.init(this)) {
            shootEnabled = false;
        }

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();

        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        panelsTelemetry.debug("Path State", currentState.name());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public class Paths {
        public final PathChain GettoArtifacts1;
        public final PathChain IntakeArtifacts1;
        public final PathChain ShootArtifacts1;
        public final PathChain GettoArtifacts2;
        public final PathChain IntakeArtifacts2;
        public final PathChain ShootArtifacts2;
        public final PathChain GettoArtifacts3;
        public final PathChain IntakeArtifacts3;
        public final PathChain ShootArtifacts3;

        public Paths(Follower follower) {
            GettoArtifacts1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.000, 12.000), new Pose(40.685, 35.451)))
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                    .build();

            IntakeArtifacts1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(40.685, 35.451), new Pose(23.972, 35.620)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ShootArtifacts1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(23.972, 35.620), new Pose(60.098, 11.986)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();

            GettoArtifacts2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(60.098, 11.986), new Pose(40.853, 60.436)))
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                    .build();

            IntakeArtifacts2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(40.853, 60.436), new Pose(24.478, 60.436)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ShootArtifacts2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(24.478, 60.436), new Pose(59.761, 12.830)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();

            GettoArtifacts3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(59.761, 12.830), new Pose(40.853, 84.070)))
                    .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                    .build();

            IntakeArtifacts3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(40.853, 84.070), new Pose(24.141, 83.564)))
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            ShootArtifacts3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(24.141, 83.564), new Pose(60.277, 12.241)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(125))
                    .build();
        }
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (currentState) {
            case SHOOT_PRELOAD:
                if (!stateStarted) {
                    shooter.launch_n_artifacts(3);
                    stateStarted = true;
                } else {
                    currentState = AutonomousState.GET_ARTIFACTS_1_PATH;
                    stateStarted = false;
                }
                break;

            case GET_ARTIFACTS_1_PATH:
                if (!stateStarted) {
                    follower.followPath(paths.GettoArtifacts1);
                    stateStarted = true;
                } else if (!follower.isBusy()) {
                    currentState = AutonomousState.INTAKE_ARTIFACTS_1_PATH;
                    stateStarted = false;
                }
                break;

            case INTAKE_ARTIFACTS_1_PATH:
                if (!stateStarted) {
                    follower.followPath(paths.IntakeArtifacts1);
                    stateStarted = true;
                } else if (!follower.isBusy()) {
                    currentState = AutonomousState.SHOOT_ARTIFACTS_1_PATH;
                    stateStarted = false;
                }
                break;

            case SHOOT_ARTIFACTS_1_PATH:
                if (!stateStarted) {
                    follower.followPath(paths.ShootArtifacts1);
                    stateStarted = true;
                } else if (!follower.isBusy()) {
                    currentState = AutonomousState.SHOOT_ARTIFACTS_1;
                    stateStarted = false;
                }
                break;

            case SHOOT_ARTIFACTS_1:
                if (!stateStarted) {
                    shooter.launch_n_artifacts(3);
                    stateStarted = true;
                } else {
                    currentState = AutonomousState.GET_ARTIFACTS_2_PATH;
                    stateStarted = false;
                }
                break;

            case GET_ARTIFACTS_2_PATH:
                if (!stateStarted) {
                    follower.followPath(paths.GettoArtifacts2);
                    stateStarted = true;
                } else if (!follower.isBusy()) {
                    currentState = AutonomousState.INTAKE_ARTIFACTS_2_PATH;
                    stateStarted = false;
                }
                break;

            case INTAKE_ARTIFACTS_2_PATH:
                if (!stateStarted) {
                    follower.followPath(paths.IntakeArtifacts2);
                    stateStarted = true;
                } else if (!follower.isBusy()) {
                    currentState = AutonomousState.SHOOT_ARTIFACTS_2_PATH;
                    stateStarted = false;
                }
                break;

            case SHOOT_ARTIFACTS_2_PATH:
                if (!stateStarted) {
                    follower.followPath(paths.ShootArtifacts2);
                    stateStarted = true;
                } else if (!follower.isBusy()) {
                    currentState = AutonomousState.SHOOT_ARTIFACTS_2;
                    stateStarted = false;
                }
                break;

            case SHOOT_ARTIFACTS_2:
                if (!stateStarted) {
                    shooter.launch_n_artifacts(3);
                    stateStarted = true;
                } else {
                    currentState = AutonomousState.GET_ARTIFACTS_3_PATH;
                    stateStarted = false;
                }
                break;

            case GET_ARTIFACTS_3_PATH:
                if (!stateStarted) {
                    follower.followPath(paths.GettoArtifacts3);
                    stateStarted = true;
                } else if (!follower.isBusy()) {
                    currentState = AutonomousState.INTAKE_ARTIFACTS_3_PATH;
                    stateStarted = false;
                }
                break;

            case INTAKE_ARTIFACTS_3_PATH:
                if (!stateStarted) {
                    follower.followPath(paths.IntakeArtifacts3);
                    stateStarted = true;
                } else if (!follower.isBusy()) {
                    currentState = AutonomousState.SHOOT_ARTIFACTS_3_PATH;
                    stateStarted = false;
                }
                break;

            case SHOOT_ARTIFACTS_3_PATH:
                if (!stateStarted) {
                    follower.followPath(paths.ShootArtifacts3);
                    stateStarted = true;
                } else if (!follower.isBusy()) {
                    currentState = AutonomousState.SHOOT_ARTIFACTS_3;
                    stateStarted = false;
                }
                break;

            case SHOOT_ARTIFACTS_3:
                if (!stateStarted) {
                    shooter.launch_n_artifacts(3);
                    stateStarted = true;
                }
                // End of autonomous sequence: could stay here or add a finished state
                break;

            default:
                // Reset or end
                currentState = AutonomousState.SHOOT_PRELOAD;
                stateStarted = false;
                break;
        }
    }
}

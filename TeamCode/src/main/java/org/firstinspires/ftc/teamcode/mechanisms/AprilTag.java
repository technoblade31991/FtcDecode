package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTag {
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final double FX = 636.967;
    private static final double FY = 636.967;
    private static final double CX = 319.933;
    private static final double CY = 251.434;
    private static final double DESIRED_DISTANCE = 30.0;
    private static final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    private static final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    private static final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    private static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    private static final double MAX_AUTO_STRAFE = 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    private static final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


    public AprilTagPoseFtc pose;
    private AprilTagProcessor aprilTag;
    private OpMode opMode;
    private MecanumDrive drive;

    /*
    Returns true if initialization was successful, else false.
    */
    public boolean init(OpMode opMode, org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive drive) {
        this.opMode = opMode;
        this.drive = drive;

        // --- Step 1: Hardware Configuration ---
        // Every webcam (like your C925) needs to be in your robot's
        // configuration file.
        // On your Driver Station, go to Configure Robot -> Control Hub -> Webcam
        // and add a webcam with the name "Webcam 1".
        WebcamName webcamName;
        try {
            webcamName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        } catch (Exception e) {
            opMode.telemetry.addData("AprilTag", "Camera initialization failed!");
            return false;
        }

        // --- Step 2: Initialize the AprilTag Processor ---
        // This is the "brains" that will do the AprilTag detection.
        // We use .Builder() to create a new processor.
        aprilTag = new AprilTagProcessor.Builder().setLensIntrinsics(FX, FY, CX, CY).build();

        // --- Step 3: Initialize the VisionPortal ---
        // This is the "eyes" that connects the camera to the processor.
        // Tell it to use your C925 ("Webcam 1")
        // Tell it to use the AprilTag processor
        // .setCameraResolution(new Size(640, 480)) // Optional
        // .setStreamFormat(VisionPortal.StreamFormat.YUY2) // Optional
        // 1. Declare your VisionPortal and AprilTagProcessor variables
        // Vision portal shows up as unused but is necessary for the camera to function.
        VisionPortal visionPortal = new VisionPortal.Builder().setCamera(webcamName).build();
        return true;
    }
    
    static class LoopResult {
        boolean targetFound;
        AprilTagDetection desiredTag;
        LoopResult(boolean targetFound, AprilTagDetection desiredTag) {
            this.targetFound = targetFound;
            this.desiredTag = desiredTag;
        }
    }

    private LoopResult loop_through_tags(Telemetry telemetry, List<AprilTagDetection> currentDetections) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata == null) {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                continue; // Skip to next detection
            }
            if (DESIRED_TAG_ID >= 0 && detection.id != DESIRED_TAG_ID) {
                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                continue; // Skip to next detection
            }
            // If checks pass, target is found
            return new LoopResult(true, detection);
        }
        return new LoopResult(false, null);
    }


    public void listen() {
        // Get a list of all AprilTags that are currently visible
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        this.opMode.telemetry.addData("# AprilTags Detected", currentDetections.size());

        if (currentDetections.isEmpty()) {
            this.opMode.telemetry.addLine("No AprilTags visible.");
        } else {
            this.opMode.telemetry.addLine("---");
        }

        // Loop through all visible tags
        LoopResult result = loop_through_tags(this.opMode.telemetry, currentDetections);

        if (result.targetFound) {
            this.opMode.telemetry.addData("\n>", "HOLD A to Drive to Target\n");
            this.opMode.telemetry.addData("Found", "ID %d (%s)", result.desiredTag.id, result.desiredTag.metadata.name);
            this.opMode.telemetry.addData("Range", "%5.1f inches", result.desiredTag.ftcPose.range);
            this.opMode.telemetry.addData("Bearing", "%3.0f degrees", result.desiredTag.ftcPose.bearing);
            this.opMode.telemetry.addData("Yaw", "%3.0f degrees", result.desiredTag.ftcPose.yaw);
        } else {
            this.opMode.telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }
        if (this.opMode.gamepad1.a && result.targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (DESIRED_DISTANCE - result.desiredTag.ftcPose.range);
            double headingError = -result.desiredTag.ftcPose.bearing;
            double yawError = result.desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double rotate = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            this.drive.driveRelativeRobot(forward, strafe, rotate, MAX_AUTO_SPEED);
        }
    }

}

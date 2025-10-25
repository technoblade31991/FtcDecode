package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private static final double DESIRED_DISTANCE = 30.0;
    private static final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    private static final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    private static final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    private static final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    private static final double MAX_AUTO_STRAFE = 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    private static final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private static final double TURN_SPEED_FIND_TAG = 0.2;   //  Turn at this speed when searching for a tag.

    public AprilTagPoseFtc pose;
    private AprilTagProcessor aprilTag;

    public void init(HardwareMap hardwareMap) {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected

        // --- Step 1: Hardware Configuration ---
        // Every webcam (like your C925) needs to be in your robot's
        // configuration file.
        // On your Driver Station, go to Configure Robot -> Control Hub -> Webcam
        // and add a webcam with the name "Webcam 1".
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // --- Step 2: Initialize the AprilTag Processor ---
        // This is the "brains" that will do the AprilTag detection.
        // We use .Builder() to create a new processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(636.967, 636.967, 319.933, 251.434)
                .build();

        // --- Step 3: Initialize the VisionPortal ---
        // This is the "eyes" that connects the camera to the processor.
        // Tell it to use your C925 ("Webcam 1")
        // Tell it to use the AprilTag processor
        // .setCameraResolution(new Size(640, 480)) // Optional
        // .setStreamFormat(VisionPortal.StreamFormat.YUY2) // Optional
        // 1. Declare your VisionPortal and AprilTagProcessor variables
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)          // Tell it to use your C925 ("Webcam 1")
                .addProcessor(aprilTag)         // Tell it to use the AprilTag processor
                // .setCameraResolution(new Size(640, 480)) // Optional
                // .setStreamFormat(VisionPortal.StreamFormat.YUY2) // Optional
                .build();
    }

    public void listen(Telemetry telemetry, Gamepad gamepad1, MecanumDrive drive) {
        boolean targetFound = false;
        AprilTagDetection desiredTag = null;
        // Get a list of all AprilTags that are currently visible
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if (currentDetections.isEmpty()) {
            telemetry.addLine("No AprilTags visible.");
        } else {
            telemetry.addLine("---");
        }

        // Loop through all visible tags
         for (AprilTagDetection detection : currentDetections) {
                        // Look to see if we have size info on this tag.
                        if (detection.metadata != null) {
                            //  Check to see if we want to track towards this tag.
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                // Yes, we want to use this tag.
                                targetFound = true;
                                desiredTag = detection;
                                break;  // don't look any further.
                            } else {
                                // This tag is in the library, but we do not want to track it right now.
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            // This tag is NOT in the library, so we don't have enough information to track to it.
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }
        if (targetFound) {
            telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }
        if (gamepad1.a && targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (DESIRED_DISTANCE - desiredTag.ftcPose.range);
            double headingError = -desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            double forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double rotate = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            drive.driveRelativeField(forward, strafe, rotate);
        }
    }

}

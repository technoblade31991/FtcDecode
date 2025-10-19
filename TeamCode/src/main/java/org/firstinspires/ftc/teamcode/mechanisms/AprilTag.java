package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Locale;

public class AprilTag {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    public AprilTagPoseFtc pose;

    public void init(HardwareMap hardwareMap) {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                // == CAMERA CALIBRATION ==
                .setLensIntrinsics(636.967, 636.967, 319.933, 251.434)
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
    }

    public void addTelemetry(Telemetry telemetry) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format(Locale.ENGLISH, "\n==== (ID %d) %s", detection.id, detection.metadata.name));
            }

            // Print pose / range / bearing if available
            if (detection.ftcPose != null) {
                this.pose = detection.ftcPose;
                double x = detection.ftcPose.x;
                double y = detection.ftcPose.y;
                double z = detection.ftcPose.z;
                double yaw = detection.ftcPose.yaw;
                double pitch = detection.ftcPose.pitch;
                double roll = detection.ftcPose.roll;
                double range = detection.ftcPose.range;
                double bearing = detection.ftcPose.bearing;

                telemetry.addData("tag " + detection.id + " x,y,z (m)", String.format(Locale.ENGLISH, "%.3f, %.3f, %.3f", x, y, z));
                telemetry.addData("tag " + detection.id + " range (m)", String.format(Locale.ENGLISH, "%.3f", range));
                telemetry.addData("tag " + detection.id + " bearing (deg)", String.format(Locale.ENGLISH, "%.1f", bearing));
                telemetry.addData("tag " + detection.id + " yaw/pitch/roll (deg)", String.format(Locale.ENGLISH, "%.1f/%.1f/%.1f",
                        Math.toDegrees(yaw), Math.toDegrees(pitch), Math.toDegrees(roll)));
            } else {
                telemetry.addData("tag " + detection.id + " pose", "null");
            }
        }

    }
}

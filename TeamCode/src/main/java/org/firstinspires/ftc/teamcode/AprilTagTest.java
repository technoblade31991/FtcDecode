package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Detection", group = "Tutorial")
public class AprilTagTest extends LinearOpMode {

    // 1. Declare your VisionPortal and AprilTagProcessor variables
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // --- Step 1: Hardware Configuration ---
        // Every webcam (like your C925) needs to be in your robot's
        // configuration file.
        // On your Driver Station, go to Configure Robot -> Control Hub -> Webcam
        // and add a webcam with the name "Webcam 1".
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "logitech");

        // --- Step 2: Initialize the AprilTag Processor ---
        // This is the "brains" that will do the AprilTag detection.
        // We use .Builder() to create a new processor.
        aprilTag = new AprilTagProcessor.Builder()
                // You can add all sorts of settings here, but
                // the defaults are usually good to start!
                 .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(636.967, 636.967, 319.933, 251.434)
                .build();

        // --- Step 3: Initialize the VisionPortal ---
        // This is the "eyes" that connects the camera to the processor.
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)          // Tell it to use your C925 ("Webcam 1")
                .addProcessor(aprilTag)         // Tell it to use the AprilTag processor
                // .setCameraResolution(new Size(640, 480)) // Optional
                // .setStreamFormat(VisionPortal.StreamFormat.YUY2) // Optional
                .build();

        // You can temporarily disable the AprilTag processor if you want
        // visionPortal.setProcessorEnabled(aprilTag, false);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "Press Play to start detecting AprilTags");
        telemetry.update();

        // Wait for the driver to press START
        waitForStart();

        while (opModeIsActive()) {

            // --- Step 4: Read Detections in the Loop ---
            telemetryAprilTag();

            // Always include a small pause in your loop
            sleep(20);
        }

        // --- Step 5: Clean up after the OpMode ends ---
        // Make sure to "close" the camera when you're done
        visionPortal.close();
    }


    /**
     * A helper method to print AprilTag data to telemetry
     */
    private void telemetryAprilTag() {

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
            // Check that the tag is one from the FTC library
            if (detection.metadata != null) {
                // Print the Tag ID and Name
                telemetry.addData("Tag", "ID %d (%s)", detection.id, detection.metadata.name);

                // Print the navigation data (Pose)
                // This is what you use to drive the robot!
                telemetry.addData("Range", "%.1f in", detection.ftcPose.range);
                telemetry.addData("Bearing", "%.1f deg", detection.ftcPose.bearing);
                telemetry.addData("Yaw", "%.1f deg", detection.ftcPose.yaw);
                telemetry.addLine("---");
            }
        }
        telemetry.update();
    }
}
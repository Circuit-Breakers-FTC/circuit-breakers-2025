package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AprilTag Webcam Test", group = "Vision")
public class AprilTagWebcamOpMode extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // Create AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Create Vision Portal using webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            telemetry.addData("Tags detected", detections.size());

            for (AprilTagDetection tag : detections) {
                telemetry.addLine("--------------------");
                telemetry.addData("ID", tag.id);
                telemetry.addData("X (in)", tag.ftcPose.x);
                telemetry.addData("Y (in)", tag.ftcPose.y);
                telemetry.addData("Z (in)", tag.ftcPose.z);
                telemetry.addData("Yaw", tag.ftcPose.yaw);
                telemetry.addData("Pitch", tag.ftcPose.pitch);
                telemetry.addData("Roll", tag.ftcPose.roll);
            }

            telemetry.update();
        }

        visionPortal.close();
    }
}

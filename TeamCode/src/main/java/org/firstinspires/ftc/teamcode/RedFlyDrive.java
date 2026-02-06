package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RedFlyAuto.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.List;

@TeleOp(name="RedFlyDrive", group="Linear OpMode")
@Config
public class RedFlyDrive extends LinearOpMode {

    public static double LAUNCH_VELOCITY = 2000;
    public static double TAG_DIST_AT_2400 = 160.0;
    public static double TAG_SLOPE = 8.0;
    public static double TAG_MIN_VEL = 1800;
    public static double TAG_MAX_VEL = 3600;

    // Auto-align parameters
    public static double ALIGN_KP = 0.02; // Proportional gain for alignment
    public static double ALIGN_MIN_POWER = 0.1; // Minimum rotation power
    public static double ALIGN_MAX_POWER = 0.3; // Maximum rotation power
    public static double ALIGN_TOLERANCE = 2.0; // Degrees tolerance for "aligned"

    private DcMotor frontRight, frontLeft, rearRight, rearLeft, intake;
    private DcMotorEx launchLeft, launchRight;
    private CRServo one, two, three, four, five, six;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ElapsedTime runtime = new ElapsedTime();

    private boolean slowMode = false;
    private boolean groupOn = false;
    private boolean autoAlign = false;

    private boolean lastXButton = false;

    public double blueAuto() {
        return 1;
    }

    @Override
    public void runOpMode() {

        // --- Hardware Map ---
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        rearRight  = hardwareMap.get(DcMotor.class, "backRight");
        rearLeft   = hardwareMap.get(DcMotor.class, "backLeft");

        intake = hardwareMap.get(DcMotor.class, "intake");
        launchLeft  = hardwareMap.get(DcMotorEx.class, "launchLeft");
        launchRight = hardwareMap.get(DcMotorEx.class, "launchRight");

        launchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        one = hardwareMap.get(CRServo.class, "one");
        two = hardwareMap.get(CRServo.class, "two");
        three = hardwareMap.get(CRServo.class, "three");
        four = hardwareMap.get(CRServo.class, "four");
        five = hardwareMap.get(CRServo.class, "five");
        six = hardwareMap.get(CRServo.class, "six");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        launchLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launchRight.setDirection(DcMotorEx.Direction.FORWARD);

        // --- AprilTag init ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        boolean intakeOn = true;
        boolean launcher = true;

        waitForStart();
        runtime.reset();

        Pose2d startPose = new Pose2d(
                new Vector2d(THIRD_PICKUP_X, PICKUP_Y),
                Math.toRadians(PICKUP_ANGLE)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Action runningAction = null;

        while (opModeIsActive()) {

            // --- GAMEPAD 1: Mecanum Drive ---
            double y = -gamepad1.left_stick_y; // forward/backward
            double x = gamepad1.left_stick_x;  // strafing
            double rx = gamepad1.right_stick_x; // rotation

            // Auto-align with right trigger
            if (gamepad1.right_trigger > 0.5) {
                autoAlign = true;
            } else {
                autoAlign = false;
            }

            // Calculate auto-align rotation if enabled
            if (autoAlign) {
                double alignRotation = getAlignmentRotation();
                // Only override manual rotation if we have a valid alignment correction
                if (alignRotation != 0) {
                    rx = alignRotation; // Override manual rotation with auto-align
                }
            }

            double frontLeftPower = y + x + rx;
            double rearLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double rearRightPower = y + x - rx;

            // Normalize powers
            double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(rearLeftPower),
                    Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));
            if (max > 1.0) {
                frontLeftPower /= max;
                rearLeftPower /= max;
                frontRightPower /= max;
                rearRightPower /= max;
            }

            // Slow mode toggle (only on button press, not hold)
            if (gamepad1.x && !lastXButton) {
                slowMode = !slowMode;
            }
            lastXButton = gamepad1.x;

            if (slowMode) {
                double slowFactor = 0.5;
                frontLeftPower *= slowFactor;
                rearLeftPower *= slowFactor;
                frontRightPower *= slowFactor;
                rearRightPower *= slowFactor;
            }

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);

            telemetry.addLine("=== DRIVER ESSENTIALS ===");
            telemetry.addData("Auto-Align", autoAlign ? "ACTIVE" : "OFF");
            telemetry.addData("Right Trigger", "%.2f", gamepad1.right_trigger);
            telemetry.addData("Rotation (rx)", "%.2f", rx);
            telemetry.addData("Drive FL/FR/RL/RR", "%.2f %.2f %.2f %.2f",
                    frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);

            // --- GAMEPAD 2: Servo Group Toggle ---
            if (gamepad2.a) {
                groupOn = true;
            }
            if (gamepad2.b) {
                groupOn = false;
            }

            if (groupOn) {
                one.setPower(1);
                two.setPower(1);
                three.setPower(1);
                four.setPower(-1);
                five.setPower(1);
                six.setPower(-1);
            } else {
                one.setPower(1);
                two.setPower(1);
                three.setPower(-1);
                four.setPower(-0.5);
                six.setPower(1);
                five.setPower(1);
            }

            // --- GAMEPAD 2: Intake Toggle ---
            if (gamepad2.right_bumper) {
                intakeOn = !intakeOn;
            }
            intake.setPower(intakeOn ? 1 : 0);

            // --- GAMEPAD 2: Launcher Toggle ---
            if (gamepad2.left_bumper) {
                launcher = !launcher;
            }

            if (launcher) {
                double shooterVel = getShooterVelocityFromAprilTag();
                launchLeft.setVelocity(shooterVel);
                launchRight.setVelocity(shooterVel);
            } else {
                launchLeft.setVelocity(0);
                launchRight.setVelocity(0);
            }

            telemetry.addData("Shooter Velocity", launchLeft.getVelocity());
            telemetry.update();
        }
    }

    // --- Helper method for auto-alignment rotation ---
    private double getAlignmentRotation() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        // Filter for only AprilTag IDs 20 and 24
        AprilTagDetection targetTag = null;
        for (AprilTagDetection detection : detections) {
            if (detection.id == 20 || detection.id == 24) {
                targetTag = detection;
                break;
            }
        }

        if (targetTag == null) {
            telemetry.addData("Align Status", "No target tag");
            return 0; // No rotation if no tag found
        }

        // Get yaw angle (horizontal angle from camera center)
        double yaw = targetTag.ftcPose.yaw;

        // Check if we're within tolerance
        if (Math.abs(yaw) < ALIGN_TOLERANCE) {
            telemetry.addData("Align Status", "LOCKED (yaw: %.1f°)", yaw);
            return 0; // Already aligned
        }

        // Calculate proportional rotation power
        double rotationPower = yaw * ALIGN_KP;

        // Apply min/max limits and ensure minimum power
        if (rotationPower > 0) {
            rotationPower = Math.max(ALIGN_MIN_POWER, Math.min(ALIGN_MAX_POWER, rotationPower));
        } else {
            rotationPower = Math.max(-ALIGN_MAX_POWER, Math.min(-ALIGN_MIN_POWER, rotationPower));
        }

        telemetry.addData("Align Status", "Adjusting (yaw: %.1f°)", yaw);
        telemetry.addData("Align Power", "%.2f", rotationPower);

        return rotationPower;
    }

    // --- Helper method for shooter velocity based on AprilTag ---
    private double getShooterVelocityFromAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        // Filter for only AprilTag IDs 20 and 24
        AprilTagDetection targetTag = null;
        for (AprilTagDetection detection : detections) {
            if (detection.id == 20 || detection.id == 24) {
                targetTag = detection;
                break;
            }
        }

        if (targetTag == null) {
            return LAUNCH_VELOCITY; // default if no matching tag found
        }

        double distance = targetTag.ftcPose.range;

        double velocity = 2400 + (distance - TAG_DIST_AT_2400) * TAG_SLOPE;
        velocity = Math.max(TAG_MIN_VEL, Math.min(TAG_MAX_VEL, velocity));

        telemetry.addData("Tag ID", targetTag.id);
        telemetry.addData("Tag Distance (in)", distance);
        telemetry.addData("Auto Shooter Vel", velocity);

        return velocity;
    }
}
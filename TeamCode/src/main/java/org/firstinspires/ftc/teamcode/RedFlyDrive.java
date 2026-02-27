package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RedFlyAuto.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
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

    public static double LAUNCH_VELOCITY = 1300;
    public static double TAG_DIST_AT_2400 = 105.0;
    public static double TAG_SLOPE = 8.333;
    public static double TAG_MIN_VEL = 900;
    public static double TAG_MAX_VEL = 3600;
    public static int cycleMotorSpeed = 6000;
    private double lastValidVelocity = LAUNCH_VELOCITY;
    public static double SPIN_MODIFIER = 1;



    private DcMotor frontRight, frontLeft, rearRight, rearLeft, intake;
    private DcMotorEx launchLeft, launchRight, cycleMotor;
    private CRServo four, five, six;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private ElapsedTime runtime = new ElapsedTime();

    private boolean slowMode = false;
    private boolean shootMode = false;
    private boolean cameraMode = false;

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

        cycleMotor = hardwareMap.get(DcMotorEx.class, "cycleMotor");

        launchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        cycleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cycleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        four = hardwareMap.get(CRServo.class, "four");
        five = hardwareMap.get(CRServo.class, "five");
        six = hardwareMap.get(CRServo.class, "six");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);

        launchLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launchRight.setDirection(DcMotorEx.Direction.FORWARD);
        cycleMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // --- AprilTag init ---
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        boolean intakeOn = true;
        boolean launcher = true;
        double shooterVel = 0;

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
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double rearLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double rearRightPower = y + x - rx;

            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(rearLeftPower),
                            Math.max(Math.abs(frontRightPower),
                                    Math.abs(rearRightPower))));
            if (max > 1.0) {
                frontLeftPower /= max;
                rearLeftPower /= max;
                frontRightPower /= max;
                rearRightPower /= max;
            }

            if (gamepad1.x) slowMode = !slowMode;
            if (slowMode) {
                frontLeftPower *= 0.5;
                rearLeftPower *= 0.5;
                frontRightPower *= 0.5;
                rearRightPower *= 0.5;
            }

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);

            // --- GAMEPAD 2: Servo Group Toggle ---
            if(gamepad2.a) {shootMode = true; cameraMode = false;}
            if(gamepad2.b) {shootMode = false; cameraMode = false;}
            if(gamepad2.right_trigger > 0.25) {shootMode = true; cameraMode = true;}

            if (shootMode) {
                four.setPower(-1);
                five.setPower(1);
                six.setPower(-1);
            } else {
                four.setPower(-0.5);
                five.setPower(1);
                six.setPower(1);

            }

            // --- Intake ---
            if (gamepad2.right_bumper) intakeOn = !intakeOn;
            intake.setPower(intakeOn ? 1 : 0);

            // --- Launcher ---
            if (gamepad2.left_bumper) launcher = !launcher;

            if (launcher) {
                launchLeft.setVelocity(cameraMode ? getLaunchVelFromTag() : LAUNCH_VELOCITY);
                launchRight.setVelocity(cameraMode ? getLaunchVelFromTag() : LAUNCH_VELOCITY);
                cycleMotor.setVelocity(cycleMotorSpeed);
            } else {
                launchLeft.setVelocity(0);
                launchRight.setVelocity(0);
                cycleMotor.setVelocity(0);
            }


            telemetry.addData("Left Vel", launchLeft.getVelocity());
            telemetry.addData("Right Vel", launchRight.getVelocity());
            telemetry.addData("Cycle Motor Vel", cycleMotor.getVelocity());
            telemetry.update();
        }
    }

    private double getLaunchVelFromTag() {

        // --- Auto-velocity from april tag ---
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            // Only use tag ID 20 or 24
            if (detection.id == 20 || detection.id == 24) {

                double distance = detection.ftcPose.range;
                double velocity = 0;

                if (distance <= 56) {
                    velocity = 8.3333 * distance + 833.3333;
                } else {
                    velocity =  10 * distance + 740;
                }


                velocity = Math.max(TAG_MIN_VEL, Math.min(TAG_MAX_VEL, velocity));

                telemetry.addData("Tag ID", detection.id);
                telemetry.addData("Tag Distance (in)", distance);
                telemetry.addData("Auto Shooter Vel", velocity);

                lastValidVelocity = velocity;   // Save it
                return velocity;
            }
        }

        // No valid tag → use last known good velocity
        return lastValidVelocity;
    }



}

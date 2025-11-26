package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.actions.CRServoAction;
import org.firstinspires.ftc.teamcode.actions.MotorActionTargetVelocity;
import org.firstinspires.ftc.teamcode.actions.MotorPowerAction;

@Autonomous
@Config
public class FlyAutonomy extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake = null;
    private DcMotorEx launchRight = null;
    private DcMotorEx launchLeft = null;
    private CRServo one = null;
    private CRServo two = null;
    private CRServo three = null;
    private CRServo four = null;
    private CRServo five = null;
    private CRServo six = null;
    private CRServo zero = null;

    public static double SHOT1_X = -15.0;
    public static double SHOT1_Y = 16.5;
    public static double SHOT1_ANGLE = 140;
    public static double FIRST_PICKUP_X = -14.0;
    public static double PICKUP_Y = 32;
    public static double PICKUP_ANGLE = 90;
    public static double FIRST_INTAKE_X = FIRST_PICKUP_X;
    public static double SECOND_PICKUP_X = 9;
    public static double SECOND_INTAKE_X = SECOND_PICKUP_X;
    public static double INTAKE_Y = 56;
    public static double START_TRAVEL_DIRECTION = 0;
    public static double END_TRAVEL_DIRECTION = 0;
    public static double LAUNCH_VELOCITY = 2000;
    public static double LAUNCH_ACCURACY = 1;

    private void runBlocking(Action a) {
        Actions.runBlocking(new ParallelAction(
                a,
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        telemetryPacket.addLine("left " + launchLeft.getVelocity());
                        telemetryPacket.addLine("right " + launchRight.getVelocity());
                        return opModeIsActive();
                    }
                }
        ));

    }
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();


        intake = hardwareMap.get(DcMotor.class, "intake");
        launchLeft = hardwareMap.get(DcMotorEx.class, "launchLeft");
        launchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launchRight = hardwareMap.get(DcMotorEx.class, "launchRight");
        launchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        one = hardwareMap.get(CRServo.class, "one");
        two = hardwareMap.get(CRServo.class, "two");
        three = hardwareMap.get(CRServo.class, "three");
        four = hardwareMap.get(CRServo.class, "four");
        five = hardwareMap.get(CRServo.class, "five");
        six = hardwareMap.get(CRServo.class, "six");
        zero = hardwareMap.get(CRServo.class, "zero");

        launchLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launchRight.setDirection(DcMotorEx.Direction.FORWARD);

        // --- INITIAL POSITIONS ---

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();        // Where we start
        Pose2d beginPose = new Pose2d(-64.5,16.5, Math.toRadians(90));

        // Bin position/drop off position
        Pose2d shotPose = new Pose2d(SHOT1_X, SHOT1_Y, Math.toRadians(SHOT1_ANGLE));
        Pose2d pickup1Pose = new Pose2d(FIRST_PICKUP_X, PICKUP_Y, Math.toRadians(90));
        Pose2d intake1Pose = new Pose2d(FIRST_INTAKE_X, INTAKE_Y, Math.toRadians(90));
        Pose2d pickup2Pose = new Pose2d(SECOND_PICKUP_X, PICKUP_Y, Math.toRadians(90));
        Pose2d intake2Pose = new Pose2d(SECOND_INTAKE_X, INTAKE_Y, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        GoBildaPinpointDriver driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        driver.resetPosAndIMU();
        waitForStart();
        telemetry.addLine("Starting");
        telemetry.update();
        runBlocking(
                new SequentialAction(
                    new ParallelAction(
                        drive.actionBuilder(beginPose)
                               .setTangent(Math.toRadians(END_TRAVEL_DIRECTION))
                                .splineToLinearHeading(shotPose, Math.toRadians(START_TRAVEL_DIRECTION))
                              .build(),
                        new MotorActionTargetVelocity(launchLeft, LAUNCH_VELOCITY, LAUNCH_ACCURACY),
                        new MotorActionTargetVelocity(launchRight, LAUNCH_VELOCITY, LAUNCH_ACCURACY),
                        new MotorPowerAction(intake, -1)
                    ),
                    new ParallelAction(
                        new CRServoAction(one, 1),
                        new CRServoAction(two, 1),
                        new CRServoAction(three, 1),
                        new CRServoAction(four, -1),
                        new CRServoAction(five, -1),
                        new CRServoAction(six, -1),
                        new CRServoAction(zero, 1)
                    ),
                    new SleepAction(6),
                    drive.actionBuilder(shotPose)
                        .setTangent(Math.toRadians(PICKUP_ANGLE))
                        .splineToLinearHeading(pickup1Pose, Math.toRadians(PICKUP_ANGLE))
                        .splineToLinearHeading(intake1Pose, Math.toRadians(PICKUP_ANGLE))
                        .splineToLinearHeading(shotPose, Math.toRadians(PICKUP_ANGLE))
                        .build(),
                    new SleepAction(4),
                    drive.actionBuilder(shotPose)
                        .setTangent(Math.toRadians(PICKUP_ANGLE))
                        .splineToLinearHeading(pickup2Pose, Math.toRadians(PICKUP_ANGLE))
                        .splineToLinearHeading(intake2Pose, Math.toRadians(PICKUP_ANGLE))
                        .splineToLinearHeading(shotPose, Math.toRadians(PICKUP_ANGLE))
                        .build(),
                        new SleepAction(6)

                        )
        );

//        One.setPower(1);
//        Two.setPower(1);
//        Three.setPower(1);
//        Four.setPower(-1);
//        Five.setPower(-1);
//        Six.setPower(-1);
//        zero.setPower(1);

        telemetry.addLine("Done");
        telemetry.update();
//        Actions.runBlocking(new SequentialAction(
//                new CRServoAction(servo, 0.5),
//                new SleepAction(3),
//                new CRServoAction(servo, 0)
//        ));
    }
}

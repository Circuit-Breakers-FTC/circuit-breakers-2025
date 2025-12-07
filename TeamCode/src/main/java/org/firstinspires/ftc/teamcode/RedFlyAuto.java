package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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

@Autonomous
@Config
public class RedFlyAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx intake = null;
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
    public static double FIRST_PICKUP_X = -12.0;
    public static double PICKUP_Y = 30;
    public static double PICKUP_ANGLE = 90;
    public static double FIRST_INTAKE_X = FIRST_PICKUP_X;
    public static double SECOND_PICKUP_X = 12;
    public static double THIRD_PICKUP_X = 36;
    public static double SECOND_INTAKE_X = SECOND_PICKUP_X;
    public static double INTAKE_Y = 56;
    public static double START_TRAVEL_DIRECTION = 180;
    public static double END_TRAVEL_DIRECTION = 180;
    public static double LAUNCH_VELOCITY = 2100;
    public static double LAUNCH_ACCURACY = 1;
    public static double INTAKE_VELOCITY = -1000;

    public static double SLEEP1 = 5;
    public static double SLEEP2 = 5;
    public static double SLEEP3 = 4.5;
    private void runBlocking(Action a) {
        Actions.runBlocking(new ParallelAction(
                a,
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        telemetryPacket.addLine("left " + launchLeft.getVelocity());
                        telemetryPacket.addLine("right " + launchRight.getVelocity());
                        telemetryPacket.addLine("intake " + intake.getVelocity());
                        return opModeIsActive();
                    }
                }
        ));

    }
    public double blueAuto(){
        return 1;
    }
    public double shotAngle() {
        return SHOT1_ANGLE;
    }
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();


        intake = hardwareMap.get(DcMotorEx.class, "intake");
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
        Pose2d beginPose = new Pose2d(62.5,16.5*blueAuto(), Math.toRadians(90*blueAuto()));

        // Bin position/drop off position
        Pose2d shotPose = new Pose2d(SHOT1_X, SHOT1_Y*blueAuto(), Math.toRadians(shotAngle()));
        Pose2d pickup1Pose = new Pose2d(FIRST_PICKUP_X, PICKUP_Y*blueAuto(), Math.toRadians(90*blueAuto()));
        Pose2d intake1Pose = new Pose2d(FIRST_INTAKE_X, INTAKE_Y*blueAuto(), Math.toRadians(90*blueAuto()));
        Pose2d pickup2Pose = new Pose2d(SECOND_PICKUP_X, PICKUP_Y*blueAuto(), Math.toRadians(90*blueAuto()));
        Pose2d intake2Pose = new Pose2d(SECOND_INTAKE_X, INTAKE_Y*blueAuto(), Math.toRadians(90*blueAuto()));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        GoBildaPinpointDriver driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        driver.resetPosAndIMU();

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();        // Where we start
        waitForStart();
        telemetry.addLine("Starting");
        telemetry.update();
        Actions.runBlocking(
                new ParallelAction(
                        // Start launchers and intake at the beginning
                        new MotorActionTargetVelocity(launchLeft, LAUNCH_VELOCITY, LAUNCH_ACCURACY),
                        new MotorActionTargetVelocity(launchRight, LAUNCH_VELOCITY, LAUNCH_ACCURACY),
                        new MotorActionTargetVelocity(intake, INTAKE_VELOCITY, 1),

                        // One continuous trajectory with markers for servo activation
                        drive.actionBuilder(beginPose)
                                // Go to shot position
                                .setTangent(Math.toRadians(END_TRAVEL_DIRECTION))
                                .splineToLinearHeading(shotPose, Math.toRadians(START_TRAVEL_DIRECTION))
                                .stopAndAdd(new ParallelAction(
                                        new CRServoAction(one, 1),
                                        new CRServoAction(two, 1),
                                        new CRServoAction(three, 1),
                                        new CRServoAction(four, -1),
                                        new CRServoAction(five, -1),
                                        new CRServoAction(six, -1),
                                        new CRServoAction(zero, 1)
                                ))
                                .waitSeconds(SLEEP1)
                                // First pickup cycle
                                .setTangent(Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeToLinearHeading(new Vector2d(FIRST_PICKUP_X, PICKUP_Y*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeTo(new Vector2d(FIRST_INTAKE_X, INTAKE_Y*blueAuto()))
                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y*blueAuto()), Math.toRadians(shotAngle()))
                                .waitSeconds(SLEEP2)
                                // Second pickup cycle
                                .setTangent(Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeToLinearHeading(new Vector2d(SECOND_PICKUP_X, PICKUP_Y*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeTo(new Vector2d(SECOND_INTAKE_X, INTAKE_Y*blueAuto()))
                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y*blueAuto()), Math.toRadians(shotAngle()))
                                .waitSeconds(SLEEP3)
                                .setTangent(Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeToLinearHeading(new Vector2d(THIRD_PICKUP_X, PICKUP_Y*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .build()
                )
        );

        telemetry.addLine("Done");
        telemetry.update();
    }
}

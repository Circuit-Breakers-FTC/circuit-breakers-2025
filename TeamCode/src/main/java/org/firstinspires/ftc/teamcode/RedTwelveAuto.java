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
import org.firstinspires.ftc.teamcode.actions.MotorPowerAction;

@Autonomous
@Config
public class RedTwelveAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx intake = null;
    private DcMotorEx launchRight = null;
    private DcMotorEx launchLeft = null;
    private DcMotorEx cycleMotor = null;

    private CRServo four = null;
    private CRServo five = null;
    private CRServo six = null;

    public static double SERVO_SPEED = 0.425;
    public static double SHOT1_X = -17.5;
    public static double SHOT1_Y = 13.5;
    public static double SHOT1_ANGLE = 135;
    public static double FIRST_PICKUP_X = -10.0;
    public static double PICKUP_Y = 26;
    public static double THIRD_PICKUP_Y = 25;
    public static double PICKUP_ANGLE = 90;
    public static double FIRST_INTAKE_X = FIRST_PICKUP_X;
    public static double SECOND_PICKUP_X = 12;
    public static double THIRD_PICKUP_X = 35;
    public static double SECOND_INTAKE_X = SECOND_PICKUP_X;
    public static double INTAKE_Y = 56;
    public static double INTAKE_Y2 = 62;
    public static double END_TRAVEL_DIRECTION = -52.7;
    public static double START_TRAVEL_DIRECTION = -52.7;
    public static double LAUNCH_VELOCITY = 1120;
    public static double LAUNCH_ACCURACY = 1;
    public static double INTAKE_VELOCITY = -1000;
    public static double TURN_BACK_ON_SERVO = 1.3;
    public static double TURN_BACK_ON_SERVO2 = 1.6;
    public static double TURN_BACK_ON_SERVO_3 = 2;
    public static double TWO_CYCLE_BACKUP_Y = 50;
    public static double START_SERVO = 1.5;

    public static double THIRDPICKUPEND = 60;
    public static double END_AUTO_Y = 13;
    public static double END_AUTO_X = -42;
    public static double END_AUTO_ANGLE = 115;

    public static double SHOOT_SLEEP1 = 2;
    public static double SHOOT_SLEEP2 = 2;
    public static double SHOOT_SLEEP3 = 2;
    public static double SHOOT_SLEEP4 = 2;
    public static int cycleMotorSpeed = 6000;
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

        cycleMotor = hardwareMap.get(DcMotorEx.class, "cycleMotor");
        cycleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cycleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cycleMotor.setDirection(DcMotorEx.Direction.FORWARD);

        four = hardwareMap.get(CRServo.class, "four");
        five = hardwareMap.get(CRServo.class, "five");
        six = hardwareMap.get(CRServo.class, "six");

        launchLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launchRight.setDirection(DcMotorEx.Direction.FORWARD);

        // --- INITIAL POSITIONS ---
        Pose2d beginPose = new Pose2d(-49.5,52*blueAuto(), Math.toRadians(127.3*blueAuto()));

        // Bin position/drop off position
        Pose2d shotPose = new Pose2d(SHOT1_X, SHOT1_Y*blueAuto(), Math.toRadians(shotAngle()));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        GoBildaPinpointDriver driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        driver.resetPosAndIMU();

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();        // Where we start
        waitForStart();
        cycleMotor.setVelocity(cycleMotorSpeed);
        telemetry.addLine("Starting");
        telemetry.update();
        Actions.runBlocking(
                new ParallelAction(
                        // Start launchers and intake at the beginning
                        new MotorActionTargetVelocity(launchLeft, LAUNCH_VELOCITY, LAUNCH_ACCURACY),
                        new MotorActionTargetVelocity(launchRight, LAUNCH_VELOCITY, LAUNCH_ACCURACY),
                        new MotorPowerAction(intake, 1),

                        // One continuous trajectory with markers for servo activation
                        drive.actionBuilder(beginPose)
                                // Go to shot position
                                .setTangent(Math.toRadians(START_TRAVEL_DIRECTION*blueAuto()))
                                .afterTime(START_SERVO,new ParallelAction(

                                        new CRServoAction(four, -1*SERVO_SPEED),
                                        new CRServoAction(five, SERVO_SPEED),
                                        new CRServoAction(six, -1*SERVO_SPEED)
                                ))
                                .splineToLinearHeading(shotPose, Math.toRadians(END_TRAVEL_DIRECTION*blueAuto()))

                                /*.stopAndAdd(new ParallelAction(
                                        new CRServoAction(one, SERVO_SPEED),
                                        new CRServoAction(two, SERVO_SPEED),
                                        new CRServoAction(three, SERVO_SPEED),
                                        new CRServoAction(four, -1*SERVO_SPEED),
                                        new CRServoAction(five, SERVO_SPEED),
                                        new CRServoAction(six, -1*SERVO_SPEED)
                                ))
                                */

                                .waitSeconds(SHOOT_SLEEP1)
                                // intake mode
                                .afterTime(0,new ParallelAction(

                                        new CRServoAction(four, -0.5),
                                        new CRServoAction(five, 1),
                                        new CRServoAction(six, 1)
                                ))
                                // First pickup cycle
                                .setTangent(Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeToLinearHeading(new Vector2d(FIRST_PICKUP_X, PICKUP_Y*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeTo(new Vector2d(FIRST_INTAKE_X, INTAKE_Y*blueAuto()))

                                .afterTime(TURN_BACK_ON_SERVO,new ParallelAction(

                                        new CRServoAction(four, -1*SERVO_SPEED),
                                        new CRServoAction(five, SERVO_SPEED),
                                        new CRServoAction(six, -1*SERVO_SPEED)
                                ))
                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y*blueAuto()), Math.toRadians(shotAngle()))
                                .waitSeconds(SHOOT_SLEEP2)
                                //second intake mode
                                .afterTime(0,new ParallelAction(

                                        new CRServoAction(four, -0.5),
                                        new CRServoAction(five, 1),
                                        new CRServoAction(six, 1)
                                ))

                                // Second pickup cycle
                                .setTangent(Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeToLinearHeading(new Vector2d(SECOND_PICKUP_X, PICKUP_Y*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeTo(new Vector2d(SECOND_INTAKE_X, INTAKE_Y2*blueAuto()))
                                .strafeTo(new Vector2d(SECOND_INTAKE_X,TWO_CYCLE_BACKUP_Y*blueAuto()))
                                .afterTime(0,new ParallelAction(

                                        new CRServoAction(four, 0),
                                        new CRServoAction(five, 0),
                                        new CRServoAction(six, 0)
                                ))
                                .afterTime(TURN_BACK_ON_SERVO2,new ParallelAction(

                                        new CRServoAction(four, -1*SERVO_SPEED),
                                        new CRServoAction(five, SERVO_SPEED),
                                        new CRServoAction(six, -1*SERVO_SPEED)
                                ))
                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y*blueAuto()), Math.toRadians(shotAngle()))
                                .waitSeconds(SHOOT_SLEEP3)
                                //third intake mode
                                .afterTime(0,new ParallelAction(

                                        new CRServoAction(four, -0.5),
                                        new CRServoAction(five, 1),
                                        new CRServoAction(six, 1)
                                ))
                                .setTangent(Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeToLinearHeading(new Vector2d(THIRD_PICKUP_X, THIRD_PICKUP_Y*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .strafeToLinearHeading(new Vector2d(THIRD_PICKUP_X, THIRDPICKUPEND*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
                                .afterTime(0,new ParallelAction(

                                        new CRServoAction(four, 0),
                                        new CRServoAction(five, 0),
                                        new CRServoAction(six, 0)
                                ))
                                .afterTime(TURN_BACK_ON_SERVO_3,new ParallelAction(

                                        new CRServoAction(four, -1*SERVO_SPEED),
                                        new CRServoAction(five, SERVO_SPEED),
                                        new CRServoAction(six, -1*SERVO_SPEED)
                                ))
                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y*blueAuto()), Math.toRadians(shotAngle()))
                                .waitSeconds(SHOOT_SLEEP4)
                                .strafeToLinearHeading(new Vector2d(END_AUTO_X, END_AUTO_Y*blueAuto()), Math.toRadians(END_AUTO_ANGLE*blueAuto()))
                                .waitSeconds(5)

                                .build()
                )
        );

        telemetry.addLine("Done");
        telemetry.update();
    }
}

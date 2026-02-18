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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.actions.ServoAction;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;


import org.firstinspires.ftc.teamcode.actions.CRServoAction;
import org.firstinspires.ftc.teamcode.actions.MotorActionTargetVelocity;
import org.firstinspires.ftc.teamcode.actions.MotorPowerAction;
import org.firstinspires.ftc.teamcode.actions.LaunchAction;

@Autonomous
@Config
public class RealSpindexerAutoCode extends LinearOpMode {
    // --- Hardware ---
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private Servo gate = null;
    private DcMotorEx spindexer = null;
    private DcMotor intake = null;
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private ElapsedTime runtime = new ElapsedTime();

    // --- Autonomous Constants / Positions ---
    public static double SERVO_SPEED = 0.425;
    public int targetPos;
    public static double SHOT1_X = -15.5;
    public static double SHOT1_Y = 14.5;
    public static double SHOT1_ANGLE = 135;
    public static double FIRST_PICKUP_X = -12.0;
    public static double PICKUP_Y = 27.5;
    public static double THIRD_PICKUP_Y = 25;
    public static double PICKUP_ANGLE = 90;
    public static double FIRST_INTAKE_X = FIRST_PICKUP_X;
    public static double SECOND_PICKUP_X = 12;
    public static double THIRD_PICKUP_X = 35;
    public static double SECOND_INTAKE_X = SECOND_PICKUP_X;
    public static double INTAKE_Y = 56;
    public static double INTAKE_Y2 = 62;
    public static double END_TRAVEL_DIRECTION = -156;
    public static double START_TRAVEL_DIRECTION = 180;
    public static double LAUNCH_VELOCITY = 2016;
    public static double LAUNCH_ACCURACY = 1;
    public static double INTAKE_VELOCITY = -1000;
    public static double TURN_BACK_ON_SERVO = 0.75;
    public static double TURN_BACK_ON_SERVO2 = 1.5;
    public static double TURN_BACK_ON_SERVO_3 = 1.5;
    public static double TWO_CYCLE_BACKUP_Y = 47;
    public static double START_SERVO = 1.5;
    public boolean intakeOn = false;

    // Spindexer additions
    final double FEED_TIME_SECONDS = 0.15; //The feeder servos run this long when a shot is requested.
    final double TIME_BETWEEN_SHOTS = 1; //Time between shots.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double gateTime = 1.0;
    double driveTime = 2;
    double speed = 1;

    boolean yWasPressed = false;
    boolean gatePressed = false;
    boolean sucking = false;
    boolean collectDriving=false;
    int v1=0;
    int pos = 1;

    int shotNumber = 0;
    boolean targetPos1 = true;
    boolean targetPos2 = true;
    boolean targetPos3 = true;
    boolean colorLocked = false;
    boolean launchNow= false;
    String shotColor1 = "empty";
    String shotColor2 = "empty";
    String shotColor3 = "empty";
    String comp1 = "green";
    String comp2 = "purple";
    String comp3 = "purple";
    String clrInComp = "empty";
    String pattern = "gpp";
    boolean index = false;

    double GATE_OPEN;
    double GATE_CLOSED;

    ElapsedTime driveTimer = new ElapsedTime();
    ElapsedTime noColorTimer = new ElapsedTime();
    ElapsedTime delayTimer = new ElapsedTime();
    private Launcher launcherSystem;





    // --- Helper Functions for Auto ---
    private void runBlocking(Action a) {
        Actions.runBlocking(new ParallelAction(
                a,
                new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        return opModeIsActive();
                    }
                }
        ));
    }

    public double blueAuto() {
        return 1;
    }

    public double shotAngle() {
        return SHOT1_ANGLE;
    }

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        ROTATE,
        LAUNCH,
        LAUNCHING,
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- Hardware Initialization ---
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        gate = hardwareMap.get(Servo.class, "gate");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "color1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color2");
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setVelocity(LAUNCH_VELOCITY);

        // --- INITIAL POSITIONS ---
        Pose2d beginPose = new Pose2d(62.5,16.5*blueAuto(), Math.toRadians(90*blueAuto()));
        Pose2d shotPose = new Pose2d(SHOT1_X, SHOT1_Y*blueAuto(), Math.toRadians(shotAngle()));
        Pose2d pickup1Pose = new Pose2d(FIRST_PICKUP_X, PICKUP_Y*blueAuto(), Math.toRadians(90*blueAuto()));
        Pose2d intake1Pose = new Pose2d(FIRST_INTAKE_X, INTAKE_Y*blueAuto(), Math.toRadians(90*blueAuto()));
        Pose2d pickup2Pose = new Pose2d(SECOND_PICKUP_X, PICKUP_Y*blueAuto(), Math.toRadians(90*blueAuto()));
        Pose2d intake2Pose = new Pose2d(SECOND_INTAKE_X, INTAKE_Y*blueAuto(), Math.toRadians(90*blueAuto()));

        // Setup MecanumDrive & Pinpoint Driver
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        GoBildaPinpointDriver driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        driver.resetPosAndIMU();

        launcherSystem = new Launcher(
                launcher,
                gate,
                LAUNCH_VELOCITY,
                LAUNCH_VELOCITY - 16,   // min velocity slightly lower
                FEED_TIME_SECONDS,
                TIME_BETWEEN_SHOTS
        );
        //==============================================================================================================================
        //CLR CODE


        double clrDiv = 1.82;
        boolean colorSeen = false;
        gate.setPosition(0.72);
        if (colorSensor1.alpha()<=colorSensor2.alpha()){
            if (colorSensor1.alpha()<=105) colorSeen = true;
            if (colorSensor1.alpha()>105) colorSeen = false;
        } else {
            if (colorSensor2.alpha()<=105) colorSeen = true;
            if (colorSensor2.alpha()>105) colorSeen = false;
        }
        if(colorSeen){
            if(colorSensor1.alpha()<=colorSensor2.alpha()){
                if(colorSensor2.green()/clrDiv>colorSensor2.red()){
                    clrInComp="green";
                } else if(colorSensor2.green()/clrDiv<colorSensor2.red()){
                    clrInComp="purple";
                }
            } else {
                if(colorSensor1.green()/clrDiv>colorSensor1.red()){
                    clrInComp="green";
                } else if(colorSensor1.green()/clrDiv<colorSensor1.red()){
                    clrInComp="purple";
                }
            }
        } else{
            //clrInComp="empty";
        }

        //==============================================================================================================================


        // --- Spindexer initial positions ---
        if (colorSeen&&!colorLocked) {
            colorLocked=true;
            noColorTimer.reset();
            delayTimer.reset();
        }
        if (colorLocked && delayTimer.seconds() >= 0.25) {
            if (pos == 1) comp1=clrInComp;
            if (pos == 2) comp2=clrInComp;
            if (pos == 3) comp3=clrInComp;
            pos++;
            if (pos > 3) pos = 1;

            colorLocked = false;
        }


        if (yWasPressed) {
            pos += 1;
            if (pos > 3) pos = 1;
            yWasPressed = false;
        }
        if (pos == 1) {
            targetPos = 0;
        } else if (pos == 2) {
            targetPos = 180;
        } else if (pos == 3) {
            targetPos = 360;
        }
        spindexer.setTargetPosition(targetPos);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(1.0);

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        waitForStart();
        telemetry.addLine("Starting");
        telemetry.update();

        // --- Main Auto Trajectory ---
        Actions.runBlocking(
                new ParallelAction(
                        new MotorPowerAction(intake, -0.9),
                        drive.actionBuilder(beginPose)
                                .setTangent(Math.toRadians(START_TRAVEL_DIRECTION*blueAuto()))
                                .afterTime(START_SERVO,new ParallelAction())
                                .splineToLinearHeading(shotPose, Math.toRadians(END_TRAVEL_DIRECTION*blueAuto()))
                                .stopAndAdd(new LaunchAction(launcherSystem))
//                                .setTangent(Math.toRadians(PICKUP_ANGLE*blueAuto()))
//                                .strafeToLinearHeading(new Vector2d(FIRST_PICKUP_X, PICKUP_Y*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
//                                .strafeTo(new Vector2d(FIRST_INTAKE_X, INTAKE_Y*blueAuto()))
//                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y*blueAuto()), Math.toRadians(shotAngle()))
//                                .stopAndAdd(new LaunchAction(launcherSystem))
//                                .setTangent(Math.toRadians(PICKUP_ANGLE*blueAuto()))
//                                .strafeToLinearHeading(new Vector2d(SECOND_PICKUP_X, PICKUP_Y*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
//                                .strafeTo(new Vector2d(SECOND_INTAKE_X, INTAKE_Y2*blueAuto()))
//                                .strafeTo(new Vector2d(SECOND_INTAKE_X,TWO_CYCLE_BACKUP_Y*blueAuto()))
//                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y*blueAuto()), Math.toRadians(shotAngle()))
//                                .stopAndAdd(new LaunchAction(launcherSystem))
//                                .setTangent(Math.toRadians(PICKUP_ANGLE*blueAuto()))
//                                .strafeToLinearHeading(new Vector2d(THIRD_PICKUP_X, THIRD_PICKUP_Y*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
//                                .strafeToLinearHeading(new Vector2d(THIRD_PICKUP_X, THIRDPICKUPEND*blueAuto()), Math.toRadians(PICKUP_ANGLE*blueAuto()))
//                                .strafeToLinearHeading(new Vector2d(END_AUTO_X, END_AUTO_Y*blueAuto()), Math.toRadians(END_AUTO_ANGLE*blueAuto()))
//                                .waitSeconds(5)
                                .build()
                )
        );

        telemetry.addLine("Done");
        telemetry.update();
        telemetry.addData("Launcher null?", launcherSystem == null);
        telemetry.update();
        sleep(2000);
    }
}

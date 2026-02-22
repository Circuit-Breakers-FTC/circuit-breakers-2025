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

import org.firstinspires.ftc.teamcode.actions.CRServoAction;
import org.firstinspires.ftc.teamcode.actions.MotorActionTargetVelocity;
import org.firstinspires.ftc.teamcode.actions.MotorPowerAction;

@Autonomous
@Config
public class RedSpindexerAuto extends LinearOpMode {
    // --- Hardware ---
    private DcMotor left_front_drive = null;
    private DcMotor right_front_drive = null;
    private DcMotor left_back_drive = null;
    private DcMotor right_back_drive = null;
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
    int pos = 1;
    boolean yWasPressed = false;

    public static double THIRDPICKUPEND = 60;
    public static double END_AUTO_Y = 8;
    public static double END_AUTO_X = -39;
    public static double END_AUTO_ANGLE = 115;
    public static double spin_Sleep = 500;
    public static double SHOOT_SLEEP1 = 2.5;
    public static double SHOOT_SLEEP2 = 3;
    public static double SHOOT_SLEEP3 = 3;

    // --- TeleOp Variables ---
    final double FEED_TIME_SECONDS = 0.15; // feeder servos run this long
    final double TIME_BETWEEN_SHOTS = 1; // time between shots
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    final double gateTime = 1.0;
    double driveTime = 2;
    double speed = 1;
    boolean yWasPressedTele = false;
    boolean gatePressed = false;
    boolean sucking = false;
    boolean collectDriving = false;
    int v1 = 0;
    String pos1_Color = "green"; // pos 1 starts with a ball
    String pos2_Color = "green"; // pos 2 starts with a ball
    String pos3_Color = "green"; //pos 3 starts with with a ball
    boolean colorLocked = false;
    boolean launchNow = false;

    String shotColor1 = "empty";
    String shotColor2 = "empty";
    String shotColor3 = "empty";
    String comp1 = "green";
    String comp2 = "purple";
    String comp3 = "purple";
    String clrInComp = "empty";
    String pattern = "gpp";
    boolean index = false;

    double LAUNCHER_TARGET_VELOCITY;
    double LAUNCHER_MIN_VELOCITY;
    double GATE_OPEN;
    double GATE_CLOSED;

    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime gateTimer = new ElapsedTime();
    ElapsedTime driveTimer = new ElapsedTime();
    ElapsedTime noColorTimer = new ElapsedTime();
    ElapsedTime delayTimer = new ElapsedTime();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        ROTATE,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

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

    private String checkColor() {
        double clrDiv = 1.82;

        ColorSensor active =
                (colorSensor1.alpha() <= colorSensor2.alpha())
                        ? colorSensor2   // colorSensor1 is dimmer → colorSensor2 is active
                        : colorSensor1;  // colorSensor2 is dimmer → colorSensor1 is active

        // No ring present
        if (active.alpha() > 105) {
            return "empty";
        }

        // Ring present — classify color
        if (active.green() / clrDiv > active.red()) {
            return "green";
        } else if (active.green() / clrDiv < active.red()) {
            return "purple";
        } else {
            return "empty"; // edge case: exactly equal
        }
    }

    private void shoot() {

        while (launcher.getVelocity() > -1400 && opModeIsActive()) {
            telemetry.addData("Launcher Speed", launcher.getVelocity());
            telemetry.addData("Status", "Spinning up...");
            telemetry.update();
        }

        // Only gets here once velocity is reached
        telemetry.addData("Status", "Ready!");
        telemetry.update();
        intake.setPower(-0.9);
        gate.setPosition(0.5);
        // wait 500ms using ElapsedTime instead of sleep()
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 500 && opModeIsActive()) {
            // just waiting
        }
        if (pos == 1) {
            pos1_Color = "empty";
        } else {
            if (pos == 2) {
                pos2_Color = "empty";
            } else {
                if (pos == 3) {
                    pos3_Color = "empty";
                }
            }
        }
        gate.setPosition(0.75);
    }

    private void go_To_Green() {
        if (pos1_Color.equals("green")) {
            pos = 1;
        }
        else if (pos2_Color.equals("green")) {
            pos = 2;
        }
        else if (pos3_Color.equals("green")) {
            pos = 3;
        }
        else if (pos1_Color.equals("purple")) {
            pos = 1;
        }
        else if (pos2_Color.equals("purple")) {
            pos = 2;
        }
        else if (pos3_Color.equals("purple")) {
            pos = 3;
        }
        else {
            telemetry.addData("giving up", pos);
            telemetry.update();
            return;
        }

        updateSpindexer();

    }

    private void go_To_Purple() {
        if (pos1_Color.equals("green")) {
            pos = 1;
        }
        else if (pos2_Color.equals("green")) {
            pos = 2;
        }
        else if (pos3_Color.equals("green")) {
            pos = 3;
        }
        else if (pos1_Color.equals("purple")) {
            pos = 1;
        }
        else if (pos2_Color.equals("purple")) {
            pos = 2;
        }
        else if (pos3_Color.equals("purple")) {
            pos = 3;
        }
        else {
            telemetry.addData("giving up", pos);
            telemetry.update();
            return;
        }

        updateSpindexer();

    }


    /* void launch(boolean shotRequested) {
         switch (launchState) {
             case IDLE:
                 if (shotRequested) launchState = LaunchState.SPIN_UP;
                 break;
             case SPIN_UP:
                 launcher.setVelocity(-LAUNCHER_TARGET_VELOCITY);
                 if (launcher.getVelocity() < -LAUNCHER_MIN_VELOCITY && feederTimer.seconds() > TIME_BETWEEN_SHOTS)
                     launchState = LaunchState.ROTATE;
                 break;
             case ROTATE:
                 launchState = LaunchState.LAUNCH;
             case LAUNCH:
                 gateTimer.reset();
                 if (v1 > 0) gatePressed = true;
                 feederTimer.reset();
                 launchState = LaunchState.LAUNCHING;
                 break;
             case LAUNCHING:
                 if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                     if (v1 == 3) v1 = 0;
                     v1 += 1;
                     launchState = LaunchState.IDLE;
                 }
                 break;
         }
     }
     */
    private void intake() {

        String detectedColor = checkColor();

        if (!detectedColor.equals("empty")) {


            if (pos == 1) {
                pos1_Color = detectedColor;
            } else if (pos == 2) {
                pos2_Color = detectedColor;
            } else if (pos == 3) {
                pos3_Color = detectedColor;
            }

            telemetry.addData("Stored in Slot", pos);
            telemetry.addData("Color", detectedColor);
            telemetry.update();
        }


    }

    private void updateSpindexer() {
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
    }

    private class IntakeAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intake();   // call your intake function every loop

            return opModeIsActive();
            // returning true keeps it running
            // returning false would stop it
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- Hardware Initialization ---
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        gate = hardwareMap.get(Servo.class, "gate");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        left_front_drive = hardwareMap.get(DcMotor.class, "left_front_drive");
        right_front_drive = hardwareMap.get(DcMotor.class, "right_front_drive");
        left_back_drive = hardwareMap.get(DcMotor.class, "left_back_drive");
        right_back_drive = hardwareMap.get(DcMotor.class, "right_back_drive");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "color1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color2");

        // --- INITIAL POSITIONS ---
        Pose2d beginPose = new Pose2d(62.5, 16.5 * blueAuto(), Math.toRadians(90 * blueAuto()));
        Pose2d shotPose = new Pose2d(SHOT1_X, SHOT1_Y * blueAuto(), Math.toRadians(shotAngle()));
        Pose2d pickup1Pose = new Pose2d(FIRST_PICKUP_X, PICKUP_Y * blueAuto(), Math.toRadians(90 * blueAuto()));
        Pose2d intake1Pose = new Pose2d(FIRST_INTAKE_X, INTAKE_Y * blueAuto(), Math.toRadians(90 * blueAuto()));
        Pose2d pickup2Pose = new Pose2d(SECOND_PICKUP_X, PICKUP_Y * blueAuto(), Math.toRadians(90 * blueAuto()));
        Pose2d intake2Pose = new Pose2d(SECOND_INTAKE_X, INTAKE_Y * blueAuto(), Math.toRadians(90 * blueAuto()));

        // Setup MecanumDrive & Pinpoint Driver
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        GoBildaPinpointDriver driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        driver.resetPosAndIMU();

        // --- Spindexer initial positions ---
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
        launcher.setVelocity(-1410);
        // --- Main Auto Trajectory ---
        Actions.runBlocking(

                new ParallelAction(
                        new IntakeAction(),  // ← THIS runs entire time
                        new MotorPowerAction(intake, -0.9),
                        drive.actionBuilder(beginPose)
                                .setTangent(Math.toRadians(START_TRAVEL_DIRECTION * blueAuto()))
                                .afterTime(START_SERVO, new ParallelAction())
                                .splineToLinearHeading(shotPose, Math.toRadians(END_TRAVEL_DIRECTION * blueAuto()))
                                //i am using purple purple green for this EX
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Green();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)//tune this vaible
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Green();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Green();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)

                                .setTangent(Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(FIRST_PICKUP_X, PICKUP_Y * blueAuto()), Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeTo(new Vector2d(FIRST_INTAKE_X, INTAKE_Y * blueAuto()))
                                //need to make code that puts the artifacts into storage during intake and intake only
                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y * blueAuto()), Math.toRadians(shotAngle()))
                                //for this example i am using purple, purple, green but i will make a april tag shooting program
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Purple();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)//tune this vaible
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Purple();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Green();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)
                                .setTangent(Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(SECOND_PICKUP_X, PICKUP_Y * blueAuto()), Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeTo(new Vector2d(SECOND_INTAKE_X, INTAKE_Y2 * blueAuto()))
                                .strafeTo(new Vector2d(SECOND_INTAKE_X, TWO_CYCLE_BACKUP_Y * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y * blueAuto()), Math.toRadians(shotAngle()))
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Purple();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)//tune this vaible
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Purple();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Green();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)
                                .setTangent(Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(THIRD_PICKUP_X, THIRD_PICKUP_Y * blueAuto()), Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(THIRD_PICKUP_X, THIRDPICKUPEND * blueAuto()), Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(END_AUTO_X, END_AUTO_Y * blueAuto()), Math.toRadians(END_AUTO_ANGLE * blueAuto()))
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Purple();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)//tune this vaible
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Purple();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)
                                .stopAndAdd(packet -> {
                                    //ex, will make camera plus aprial tag system
                                    go_To_Green();
                                    shoot();
                                    return false;
                                })
                                .waitSeconds(spin_Sleep)

                                .build()
                )
        );

        telemetry.addLine("Done");
        telemetry.update();
    }
}

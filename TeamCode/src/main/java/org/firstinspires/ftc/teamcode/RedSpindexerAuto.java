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
   
    private DcMotorEx launcher = null;
    private Servo gate = null;
    private DcMotorEx spindexer = null;
    private DcMotor intake = null;
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private ElapsedTime runtime = new ElapsedTime();
    ElapsedTime delayTimer = new ElapsedTime();

    // --- Autonomous Constants / Positions ---

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

    public static double TWO_CYCLE_BACKUP_Y = 47;
    public static double START_SERVO = 1.5;
    public boolean intakeOn = false;
    public boolean colorSeen = false;

    // Spindexer additions
    int pos = 1;
    boolean yWasPressed = false;

    public static double THIRDPICKUPEND = 60;
    public static double END_AUTO_Y = 8;
    public static double END_AUTO_X = -39;
    public static double END_AUTO_ANGLE = 115;
    public static double spin_Sleep = 1;


    // --- TeleOp Variables ---

    String pos1_Color = "green"; // pos 1 starts with a ball
    String pos2_Color = "green"; // pos 2 starts with a ball
    String pos3_Color = "green"; //pos 3 starts with with a ball


    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime gateTimer = new ElapsedTime();
    ElapsedTime driveTimer = new ElapsedTime();
    ElapsedTime noColorTimer = new ElapsedTime();

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
        intake.setPower(-1.0);
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 750 && opModeIsActive()) {
            // just waiting
        }
        gate.setPosition(0.5);
        // wait 500ms using ElapsedTime instead of sleep()
        timer.reset();
        while (timer.milliseconds() < 1250 && opModeIsActive()) {
            // just waiting
        }
        gate.setPosition(0.75);
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
                ElapsedTime timer = new ElapsedTime();
                while (timer.milliseconds() < 250 && opModeIsActive()) {
                    // just waiting
                }
                pos1_Color = detectedColor;
                telemetry.addData("Stored in Slot", pos);
                telemetry.addData("Color", detectedColor);
                telemetry.update();
            } else if (pos == 2) {
                ElapsedTime timer = new ElapsedTime();
                while (timer.milliseconds() < 250 && opModeIsActive()) {
                    // just waiting
                }
                pos2_Color = detectedColor;
                telemetry.addData("Stored in Slot", pos);
                telemetry.addData("Color", detectedColor);
                telemetry.update();
            } else if (pos == 3) {
                ElapsedTime timer = new ElapsedTime();
                while (timer.milliseconds() < 250 && opModeIsActive()) {
                    // just waiting
                }
                pos3_Color = detectedColor;
                telemetry.addData("Stored in Slot", pos);
                telemetry.addData("Color", detectedColor);
                telemetry.update();
            }
        }
        telemetry.addData("pos 1 color", pos1_Color);
        telemetry.addData("pos 2 color", pos2_Color);
        telemetry.addData("pos 3 color", pos3_Color);
        telemetry.update();

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
    private boolean autoRotateEnabled = false;

    private void ballSensor() {
        if (colorSensor1.alpha()<=colorSensor2.alpha()){
            if (colorSensor1.alpha()<=105) colorSeen = true;
            if (colorSensor1.alpha()>105) colorSeen = false;
        } else {
            if (colorSensor2.alpha()<=105) colorSeen = true;
            if (colorSensor2.alpha()>105) colorSeen = false;
        }
    }

//    private void spindexerRotateIntake() {
//        if (colorSeen&&!colorLocked) {
//            colorLocked=true;
//            noColorTimer.reset();
//            delayTimer.reset();
//        }
//        if (colorLocked && delayTimer.seconds() >= 0.25) {
//            pos++;
//            if (pos > 3) pos = 1;
//
//            colorLocked = false;
//        }
//
//    }false
    private void autoRotateToEmpty() {
        if (!autoRotateEnabled) return;

        // Check if current pos slot is empty, if not rotate to find one
        for (int i = 0; i < 3; i++) {
            String currentColor;
            if (pos == 1) currentColor = pos1_Color;
            else if (pos == 2) currentColor = pos2_Color;
            else currentColor = pos3_Color;

            if (currentColor.equals("empty")) {
                updateSpindexer(); // already on an empty slot, lock in
                return;
            }

            // Current slot is full, try next
            pos += 1;
            if (pos > 3) pos = 1;
        }

        // All slots are full, no empty slot found
        telemetry.addData("Spindexer", "All slots full, nowhere to rotate");
        telemetry.update();
    }


    private class IntakeAction implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            autoRotateToEmpty();
            intake();   // call your intake function every loop
            ballSensor();
            //spindexerRotateIntake();

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
                        new IntakeAction(),  // THIS runs entire time
                        new MotorPowerAction(intake, -0.9),
                        drive.actionBuilder(beginPose)
                                .setTangent(Math.toRadians(START_TRAVEL_DIRECTION * blueAuto()))
                                .afterTime(START_SERVO, new ParallelAction())
                                .splineToLinearHeading(shotPose, Math.toRadians(END_TRAVEL_DIRECTION * blueAuto()))
                                .stopAndAdd(packet -> {
                                    autoRotateEnabled = false; //with this it will make it so the auto rotate does not do wierd stuff
                                    return false;
                                })
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
                                .stopAndAdd(packet -> {
                                    autoRotateEnabled = true; //with this it will make it so it can pick up and store artifacts
                                    return false;
                                })
                                .setTangent(Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(FIRST_PICKUP_X, PICKUP_Y * blueAuto()), Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeTo(new Vector2d(FIRST_INTAKE_X, INTAKE_Y * blueAuto()),new MaxVelocity(10))
                                //need to make code that puts the artifacts into storage during intake and intake only
                                .strafeToLinearHeading(new Vector2d(SHOT1_X, SHOT1_Y * blueAuto()), Math.toRadians(shotAngle()))
                                .stopAndAdd(packet -> {
                                    autoRotateEnabled = false; //with this it will make it so the auto rotate does not do wierd stuff
                                    return false;
                                })
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
                                .stopAndAdd(packet -> {
                                    autoRotateEnabled = true; //with this it will make it so it can pick up and store artifacts
                                    return false;
                                })
                                .setTangent(Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(SECOND_PICKUP_X, PICKUP_Y * blueAuto()), Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeTo(new Vector2d(SECOND_INTAKE_X, INTAKE_Y2 * blueAuto()),new MaxVelocity(10))
                                .strafeTo(new Vector2d(SECOND_INTAKE_X, TWO_CYCLE_BACKUP_Y * blueAuto()))
                                .stopAndAdd(packet -> {
                                    autoRotateEnabled = false; //with this it will make it so the auto rotate does not do wierd stuff
                                    return false;
                                })
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
                                .waitSeconds(spin_Sleep)
                                .stopAndAdd(packet -> {
                                    autoRotateEnabled = true; //with this it will make it so it can pick up and store artifacts
                                    return false;
                                })
                                .setTangent(Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(THIRD_PICKUP_X, THIRD_PICKUP_Y * blueAuto()), Math.toRadians(PICKUP_ANGLE * blueAuto()))
                                .strafeToLinearHeading(new Vector2d(THIRD_PICKUP_X, THIRDPICKUPEND * blueAuto()), Math.toRadians(PICKUP_ANGLE * blueAuto()),new MaxVelocity(10))
                                .strafeToLinearHeading(new Vector2d(END_AUTO_X, END_AUTO_Y * blueAuto()), Math.toRadians(END_AUTO_ANGLE * blueAuto()))
                                .stopAndAdd(packet -> {
                                    autoRotateEnabled = true; //with this it will make it so it can pick up and store artifacts
                                    return false;
                                })
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

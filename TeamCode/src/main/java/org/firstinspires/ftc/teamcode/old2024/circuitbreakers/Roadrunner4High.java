package org.firstinspires.ftc.teamcode.old2024.circuitbreakers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.actions.CRServoAction;
import org.firstinspires.ftc.teamcode.MaxVelocity;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.MotorAction;

@Autonomous(name = "Roadrunner4High")
@Config
@Disabled
public final class Roadrunner4High extends LinearOpMode {
    private DcMotor lift;
    private DcMotor armTurn;
    private CRServo intake;

    // Deposit Inputs
    public static double DEPOSIT_POSITION_X = 21;
    public static double DEPOSIT_POSITION_Y = 127.5;
    public static double DEPOSIT_POSITION_HEADING = 135;
    public static double DEPOSIT_POSITION_DIRECTION = 75;

    // Push Inputs
    public static double PUSH_POSITION_END_X = 50;
    public static double PUSH_POSITION_END_Y = 115;
    public static double PUSH_POSITION_HEADING = 80;
    public static double PUSH_POSITION_DIRECTION = 180;

    // Pickup Position
    public static double PICKUP_POSITION_X = 48;
    public static double PICKUP_POSITION_Y = 105;
    public static double PICKUP_POSITION_HEADING = 90;
    public static double PICKUP_POSITION_DIRECTION = -20;
    public static double BEFORE_4_POSE_DIRECTION = 0;

    // Pick up Position 2
    public static double PICKUP_POSITION_Y2 = 115;

    // Pick up Position 3
    public static double PICKUP_POSITION_Y3 = 123;

    // Pick up Position 4
    public static double PICKUP_POSITION_Y4 = 108;

    // Pick up Last Sample Position 5
    public static double PICKUP_POSITION_Y5 = 128;

    // Go to before Position 4
    public static double PICKUP_POSITION_X2 = 36;

    // Set the Variables
    public static int ARM_FIRST_SAMPLE = -3032;
    public static int LIFT_FIRST_SAMPLE_UP = -750;
    public static int LIFT_FIRST_SAMPLE_DOWN = -300;
    public static int ARM_PICKUP = -2790;
    public static int ARM_BIN =-1490;
    public static double ARM_POWER = 1.0;
    public static int ARM_START = 0;
    public static int ARM_MIDDLE = -1000;
    public static int LIFT_START = -5;
    public static double LIFT_POWER = 1.0;
    public static double DEPOSIT_SLEEP_TIME = 0.5;
    public static double DEPOSIT_SLEEP_TIME_PRE = 0.75;

    public static double BAR_POSITION_X = 65;
    public static double BAR_POSITION_Y = 102;//95;
    public static double BAR_HEADING = -90;

    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTurn = hardwareMap.get(DcMotor.class, "armTurn");
        armTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(CRServo.class, "intake");

        // Where we start
        Pose2d beginPose = new Pose2d(8, 4*24+8.5, Math.toRadians(90));

        // Bin position/drop off position
        Pose2d depositPose = new Pose2d(DEPOSIT_POSITION_X, DEPOSIT_POSITION_Y, Math.toRadians(DEPOSIT_POSITION_HEADING));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();

        // Add line to driver hub
        telemetry.addLine("Starting");
        telemetry.update();

        // The Start of the auto
        Actions.runBlocking(
                new SequentialAction(

                        // First sample(The one we already had loaded into the robot at the start)
                        new ParallelAction(
                                new MotorAction(armTurn,ARM_BIN, ARM_POWER),
                                new MotorAction(lift, -1700, LIFT_POWER),
                                drive.actionBuilder(beginPose)
                                        .setTangent(0)
                                        .splineToLinearHeading(depositPose, Math.toRadians(DEPOSIT_POSITION_DIRECTION))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(DEPOSIT_SLEEP_TIME_PRE),
                                        new CRServoAction(intake, 0.35)//first deposit
                                )

                        ),
                        new SleepAction(DEPOSIT_SLEEP_TIME),


                        // Pick up the First Sample
                        binTobin(drive, depositPose, 0),
                        binTobin(drive,depositPose, 10),
                        binTobin4thSample(drive, depositPose,8),
                        bar(drive,depositPose, 0)



                )
        );

        armTurn.setPower(0);

        // Add line "Finish"
        telemetry.addLine("Finished");
        telemetry.update();
    }

    public SequentialAction binTobin4thSample(MecanumDrive drive, Pose2d depositPose, double offset) {
        // Pick up the First Sample
        Pose2d pickupPose4 = new Pose2d(PICKUP_POSITION_X,PICKUP_POSITION_Y4+offset,Math.toRadians((PICKUP_POSITION_HEADING)));

        // Go to before pick up of the first sample
        Pose2d before4Pose = new Pose2d(PICKUP_POSITION_X2,PICKUP_POSITION_Y4+offset, Math.toRadians(PICKUP_POSITION_HEADING));

        Pose2d driveToPose4 = new Pose2d(PICKUP_POSITION_X,PICKUP_POSITION_Y5,Math.toRadians(PICKUP_POSITION_HEADING));


        return
                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        new ParallelAction(
                                                new MotorAction(lift,LIFT_FIRST_SAMPLE_UP, 0.5),
                                                new MotorAction(armTurn,ARM_FIRST_SAMPLE,0.5)
                                        )
                                ),
                                new CRServoAction(intake, -0.3),
                                drive.actionBuilder(depositPose)
                                        .setTangent(Math.toRadians(-45))
                                        .splineToLinearHeading(before4Pose, Math.toRadians(0), new MaxVelocity(20))
                                        .splineToLinearHeading(pickupPose4,Math.toRadians(0), new MaxVelocity(20))
                                        .splineToLinearHeading(driveToPose4,Math.toRadians(0), new MaxVelocity(20))
                                        .build()

                        ),





                        new MotorAction(lift, LIFT_FIRST_SAMPLE_DOWN, 0.5),
                        new SleepAction(0.75),
                        new ParallelAction(
                                new MotorAction(armTurn,ARM_BIN, ARM_POWER),
                                new MotorAction(lift, -1700, LIFT_POWER),
                                drive.actionBuilder(pickupPose4)
                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(depositPose, Math.toRadians(DEPOSIT_POSITION_DIRECTION))
                                        .build()

                        ),
                        new CRServoAction(intake, 0.35),    //deposit 4
                        new SleepAction(0.75)

                );
    }
    public SequentialAction binTobin(MecanumDrive drive, Pose2d depositPose, double offset) {
        // Pick up the First Sample
        Pose2d pickupPose = new Pose2d(PICKUP_POSITION_X,PICKUP_POSITION_Y4+offset,Math.toRadians((PICKUP_POSITION_HEADING)));

        // Go to before pick up of the first sample
        Pose2d before4Pose = new Pose2d(PICKUP_POSITION_X2,PICKUP_POSITION_Y4+offset, Math.toRadians(PICKUP_POSITION_HEADING));


        return
                new SequentialAction(
                        new ParallelAction(
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        new ParallelAction(
                                            new MotorAction(lift,LIFT_FIRST_SAMPLE_UP, 0.5),
                                            new MotorAction(armTurn,ARM_FIRST_SAMPLE,0.5)
                                        )
                                ),
                                new CRServoAction(intake, -0.3),
                                drive.actionBuilder(depositPose)
                                        .setTangent(Math.toRadians(-45))
                                        .splineToLinearHeading(before4Pose, Math.toRadians(0), new MaxVelocity(20))
                                        .splineToLinearHeading(pickupPose,Math.toRadians(0), new MaxVelocity(20))
                                        .build()

                        ),





                        new MotorAction(lift, LIFT_FIRST_SAMPLE_DOWN, 0.5),
                        new SleepAction(0.75),
                        new ParallelAction(
                                new MotorAction(armTurn,ARM_BIN, ARM_POWER),
                                new MotorAction(lift, -1700, LIFT_POWER),
                                drive.actionBuilder(pickupPose)
                                        .setTangent(Math.toRadians(180))
                                        .splineToLinearHeading(depositPose, Math.toRadians(DEPOSIT_POSITION_DIRECTION), new MaxVelocity(20))
                                        .build()

                        ),
                        new CRServoAction(intake, 0.35),    //deposit 2,3
                        new SleepAction(0.75)

                );
    }
    public SequentialAction bar(MecanumDrive drive, Pose2d depositPose, double offset) {
        // Bar pose midpoint
        Pose2d barPose1 = new Pose2d(60,119+offset,Math.toRadians((90)));

        // Bar pose end
        Pose2d barPose = new Pose2d(BAR_POSITION_X,BAR_POSITION_Y, Math.toRadians(BAR_HEADING));



        return
                new SequentialAction(
                        new ParallelAction(
                                new MotorAction(lift,LIFT_START, 1),
                                new MotorAction(armTurn,-1600,0.5),
                                new CRServoAction(intake, 0),
                                drive.actionBuilder(depositPose)
                                        .setTangent(Math.toRadians(0))
                                        //.splineToLinearHeading(barPose1, Math.toRadians(0))
                                        .splineToLinearHeading(barPose, Math.toRadians(-90), new MaxVelocity(500))
                                        .build(),
                                new SequentialAction(
                                        new SleepAction(2.0),
                                        new MotorAction(armTurn,-1988,0.5)
                                )

                        )








                );

    }

}


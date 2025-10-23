package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.actions.CRServoAction;
import org.firstinspires.ftc.teamcode.actions.MotorAction;
import org.firstinspires.ftc.teamcode.actions.MotorActionTargetVelocity;

@Autonomous
@Config
public class BlueAuto extends LinearOpMode {
    private CRServo servo = null;
    public static double SHOT_X = -36.0;
    public static double SHOT_Y = 12;
    public static double SHOT_ANGLE = 135;
    public static double START_TRAVEL_DIRECTION = 180;
    public static double END_TRAVEL_DIRECTION = 180;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "servo");
        // Where we start
        Pose2d beginPose = new Pose2d(0,0, Math.toRadians(0));

        // Bin position/drop off position
        Pose2d shotPose = new Pose2d(SHOT_X, SHOT_Y, Math.toRadians(SHOT_ANGLE));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        waitForStart();
        telemetry.addLine("Starting");
        telemetry.update();
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(beginPose)
                                        .setTangent(Math.toRadians(END_TRAVEL_DIRECTION))
                                        .splineToLinearHeading(shotPose, Math.toRadians(START_TRAVEL_DIRECTION))
                                        .build(),
                                new CRServoAction(servo, 0.5)
                        ),
                        new SleepAction(3),
                        new CRServoAction(servo, 0)
                )
        );
        telemetry.addLine("Done");
        telemetry.update();
//        Actions.runBlocking(new SequentialAction(
//                new CRServoAction(servo, 0.5),
//                new SleepAction(3),
//                new CRServoAction(servo, 0)
//        ));
    }
}


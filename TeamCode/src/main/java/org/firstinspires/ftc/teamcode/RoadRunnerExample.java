package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Config
public class RoadRunnerExample extends LinearOpMode {
    public static double SHOT_X = -36.0;
    public static double SHOT_Y = 12;
    public static double SHOT_ANGLE = 135;
    public static double START_TRAVEL_DIRECTION = 180;
    public static double END_TRAVEL_DIRECTION = 180;
    @Override
    public void runOpMode() throws InterruptedException {
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
                                        .setTangent(Math.toRadians(START_TRAVEL_DIRECTION))
                                        .splineToLinearHeading(shotPose, Math.toRadians(END_TRAVEL_DIRECTION))
                                        .build()
                        )
                )
        );

    }
}

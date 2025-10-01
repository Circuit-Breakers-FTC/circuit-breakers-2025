package org.firstinspires.ftc.teamcode.old2024.circuitbreakers;

import static org.firstinspires.ftc.teamcode.old2024.circuitbreakers.Roadrunner4High.ARM_POWER;
import static org.firstinspires.ftc.teamcode.old2024.circuitbreakers.Roadrunner4High.BAR_HEADING;
import static org.firstinspires.ftc.teamcode.old2024.circuitbreakers.Roadrunner4High.BAR_POSITION_X;
import static org.firstinspires.ftc.teamcode.old2024.circuitbreakers.Roadrunner4High.BAR_POSITION_Y;
import static org.firstinspires.ftc.teamcode.old2024.circuitbreakers.Roadrunner4High.DEPOSIT_POSITION_DIRECTION;
import static org.firstinspires.ftc.teamcode.old2024.circuitbreakers.Roadrunner4High.DEPOSIT_POSITION_HEADING;
import static org.firstinspires.ftc.teamcode.old2024.circuitbreakers.Roadrunner4High.DEPOSIT_POSITION_X;
import static org.firstinspires.ftc.teamcode.old2024.circuitbreakers.Roadrunner4High.DEPOSIT_POSITION_Y;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MaxVelocity;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.actions.MotorAction;

import java.util.ArrayList;
import java.util.List;


//programmed by gabi and theo
@TeleOp(name = "MainDrive")
@Disabled
public class MainDrive extends LinearOpMode {

    public static double LIFT_POWER = 1.0;
    private List<Action> runningActions = new ArrayList<>();
    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor lift;
    private DcMotor armTurn;
    private CRServo intake;
    private Servo extendArm;
    private double ticksPerRotation;
    private void resetTarget() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // armTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    static final double COUNTS_PER_MOTOR_REV = 537.6;    // eg: TETRIX Motor Encoder

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        lift = hardwareMap.get(DcMotor.class, "lift");
  //      lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armTurn = hardwareMap.get(DcMotor.class, "armTurn");
        //armTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //  armTurn.setTargetPosition(0);
        armTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(CRServo.class, "intake");
        extendArm = hardwareMap.get(Servo.class, "extendArm");
        //     Initialization for NEW robot
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        extendArm.setPosition(0.4);
        pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
        blinkinLedDriver.setPattern(pattern);
        boolean isLaunched = false;
        boolean fastMode = false;
        // that's the fast mode ;)
        Pose2d barPose = new Pose2d(BAR_POSITION_X,BAR_POSITION_Y, Math.toRadians(BAR_HEADING));

        MecanumDrive drive = new MecanumDrive(hardwareMap, barPose);



        waitForStart();
        Action runningAction = null;
        Pose2d depositPose = new Pose2d(DEPOSIT_POSITION_X, DEPOSIT_POSITION_Y, Math.toRadians(DEPOSIT_POSITION_HEADING));

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            boolean i = gamepad1.b;
            //   directions
            int target = 0;
            if (gamepad1.right_bumper) {
                fastMode = true;
            } else {
                fastMode = false;
                telemetry.addLine("left bumper for fast mode!");

            }

            if (gamepad2.right_bumper && gamepad2.left_bumper) {
                resetTarget();
                //resets encoders
            }

            //Lifter
            if (gamepad2.a) {//ground
                lift.setTargetPosition(0);
                lift.setPower(1);
            } else if (gamepad2.x) {
                lift.setPower(0);
            } else if (gamepad2.b) {//high basket
                lift.setTargetPosition(-1700);
                lift.setPower(1);
            } else if (gamepad2.y) {//low basket
                lift.setTargetPosition(-1000);
                lift.setPower(1);
            }
            //Intake
            if (gamepad2.dpad_down) {
                intake.setPower(0.25);
                pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                blinkinLedDriver.setPattern(pattern);
                //push out
            }

            if (gamepad2.dpad_up) {
                intake.setPower(-0.5);
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                blinkinLedDriver.setPattern(pattern);
                //suck in
            }

            if (gamepad2.dpad_left) {
                intake.setPower(0);
                pattern = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
                blinkinLedDriver.setPattern(pattern);
                //stop
            }
            //Arm code
            if (gamepad1.y) { //basket
                armTurn.setTargetPosition(-1490);
                armTurn.setPower(0.5);

            } else if (gamepad1.b) {//hover
                armTurn.setTargetPosition(-2790);
                armTurn.setPower(0.5);
                //too low, changed it -6
            } else if (gamepad1.x) {//drive
                armTurn.setTargetPosition(-2490);
                armTurn.setPower(0.5);
            } else if (gamepad1.a) {//floor
                armTurn.setTargetPosition(-2850);
                armTurn.setPower(0.25);
            }


            //calculates/sets drive input
            double x = gamepad1.left_stick_x - gamepad1.left_trigger + gamepad1.right_trigger; // Strafe left/right
            double y = -gamepad1.left_stick_y; // Forward/backward
            double rotation = gamepad1.right_stick_x; // Rotate


            // Calculate motor powers
            double frontLeftPower = y + x + rotation;
            double frontRightPower = y - x - rotation;
            double backLeftPower = y - x + rotation;
            double backRightPower = y + x - rotation;
            double slowness = (.5);
            // Normalize motor powers to ensure they don't exceed 1.0
            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            if (fastMode) {
                slowness = 1.0;
            }
            TelemetryPacket packet = new TelemetryPacket();
            if (gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left) {
                // automatic drive mode
                if (gamepad1.dpad_up && runningAction == null) {
                    runningAction = new ParallelAction(
//
                            new MotorAction(lift, -1700, LIFT_POWER),
                            new MotorAction(armTurn, -1490, ARM_POWER),

                            drive.actionBuilder(drive.localizer.getPose())
                                    .setTangent(Math.toRadians(90))
                                    .splineToLinearHeading(depositPose, Math.toRadians(DEPOSIT_POSITION_DIRECTION), new VelConstraint() {
                                        @Override
                                        public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                                            return 100;
                                        }
                                    })
                                    .build()
                    );
                }
                if (gamepad1.dpad_down && runningAction == null) {
                    Pose2d samplePickup = new Pose2d (BAR_POSITION_X,120,Math.toRadians(-90));
                    runningAction = new ParallelAction(
                            new SequentialAction(
                                    new SleepAction(0.5),
                                        new ParallelAction(
                                            new MotorAction(lift, 0, LIFT_POWER),
                                            new MotorAction(armTurn, -2490, ARM_POWER)
                                        )
                            ),

                            drive.actionBuilder(drive.localizer.getPose())
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(samplePickup, Math.toRadians(-90), new VelConstraint() {
                                        @Override
                                        public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
                                            return 100;
                                        }
                                    })
                                    .build()
                    );
                }
                if (gamepad1.dpad_left && runningAction == null) {
                    Pose2d robotPark = new Pose2d (12, 24,Math.toRadians(90));
                    runningAction = new ParallelAction(
                            drive.actionBuilder(drive.localizer.getPose())
                                    .setTangent(Math.toRadians(-90))
                                    .splineToLinearHeading(robotPark, Math.toRadians(-90),new MaxVelocity(1000))
                                    .build()
                    );
                }
                //runningAction.preview(packet.fieldOverlay());
                if (runningAction.run(packet)) {
                    telemetry.addLine("Roadrunner action still running");
                } else {
                    telemetry.addLine("Roadrunner action finished");
                }
            } else {
                // manual drive
                runningAction = null;
                frontLeft.setPower(frontLeftPower * slowness);
                backLeft.setPower(backLeftPower * slowness);
                frontRight.setPower(frontRightPower * slowness);
                backRight.setPower(backRightPower * slowness);
                telemetry.addData("y ", y);
                telemetry.addData("x ", x);
                telemetry.addData("fastMode", fastMode);
                telemetry.addData("armLift Current Position", armTurn.getCurrentPosition());
                telemetry.addData("lift Current Position", lift.getCurrentPosition());
                telemetry.addData("Target Position", lift.getTargetPosition());
                telemetry.addData("Unfold Arm Current Position", extendArm.getPosition());
            }


            telemetry.addData("pose x", drive.localizer.getPose().position.x);
            telemetry.addData("pose y", drive.localizer.getPose().position.y);
            telemetry.addData("heading", drive.localizer.getPose().heading);
            telemetry.update();

        }
    }
}






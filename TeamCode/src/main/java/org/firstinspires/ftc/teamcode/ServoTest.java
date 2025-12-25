package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RedFlyAuto.PICKUP_ANGLE;
import static org.firstinspires.ftc.teamcode.RedFlyAuto.PICKUP_Y;
import static org.firstinspires.ftc.teamcode.RedFlyAuto.SHOT1_ANGLE;
import static org.firstinspires.ftc.teamcode.RedFlyAuto.SHOT1_X;
import static org.firstinspires.ftc.teamcode.RedFlyAuto.SHOT1_Y;
import static org.firstinspires.ftc.teamcode.RedFlyAuto.THIRD_PICKUP_X;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Config
public class ServoTest extends LinearOpMode {

    public static double PARK_X = 39.0;
    public static double PARK_Y = -33.0;
    public static double PARK_ANGLE = 90;
    public static double LAUNCH_VELOCITY = 2000;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor rearRight = null;
    private DcMotor rearLeft = null;
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

    public double blueAuto(){
        return 1;
    }
    @Override
    public void runOpMode() {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- HARDWARE MAPPING ---
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        rearRight  = hardwareMap.get(DcMotor.class, "backRight");
        rearLeft   = hardwareMap.get(DcMotor.class, "backLeft");

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

        // --- MOTOR DIRECTIONS ---
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.REVERSE);
        launchLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launchRight.setDirection(DcMotorEx.Direction.FORWARD);

        // --- INITIAL POSITIONS ---
        boolean groupOn = false;
        boolean intakeOn = true;
        boolean launcher = true;
        boolean slowMode = false;

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();
        Pose2d startPose = new Pose2d(new Vector2d(THIRD_PICKUP_X, PICKUP_Y * blueAuto()), Math.toRadians(PICKUP_ANGLE * blueAuto()));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Action runningAction = null;

        while (opModeIsActive()) {
            if (gamepad1.a)
                one.setPower(1);
            else {
                one.setPower(0);
            }
            if (gamepad1.b)
                two.setPower(1);
            else {
                two.setPower(0);
            }
            if (gamepad1.x)
                three.setPower(1);
            else {
                three.setPower(0);
            }
            if (gamepad1.y)
                four.setPower(1);
            else {
                four.setPower(0);
            }
            if (gamepad2.a)
                five.setPower(1);
            else {
                five.setPower(0);
            }
            if (gamepad2.b)
                six.setPower(1);
            else {
                six.setPower(0);
            }
            if (gamepad2.x)
                zero.setPower(1);
            else {
                zero.setPower(0);
            }


            // --- DRIVER ESSENTIALS ---
            Pose2d pose = drive.localizer.getPose();
            telemetry.addData("position x", pose.position.x);
            telemetry.addData("position y", pose.position.y);
            telemetry.addData("heading ", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("slow mode (x button)", slowMode);
            telemetry.addData("slow mode (x button)", slowMode);
            telemetry.addData("Intake (right bumper)", intakeOn ? "ON" : "OFF");
            telemetry.addData("Launcher (left bumper)", launcher ? "ON" : "OFF");
            telemetry.addData("Servo Group (A button)", groupOn ? "ON" : "OFF");
            // --- EXTRA INFO ---
            telemetry.addLine("--- EXTRA INFO ---");
            telemetry.addData("Launcher Vel L/R", "%.0f/%.0f", launchLeft.getVelocity(), launchRight.getVelocity());
            telemetry.addData("Joystick Y/X/RX", "%.2f %.2f %.2f", -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            telemetry.addData("Individual Servo Powers", "One: %.2f Two: %.2f Three: %.2f Four: %.2f Six: %.2f Zero: %.2f",
                    one.getPower(), two.getPower(), three.getPower(), four.getPower(), six.getPower(), zero.getPower());
            telemetry.addData("Runtime", runtime.toString());

            telemetry.update();
        }
    }
}

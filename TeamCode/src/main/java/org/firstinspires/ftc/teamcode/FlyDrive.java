package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="FlyDrive", group="Linear OpMode")
public class FlyDrive extends LinearOpMode {

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

    @Override
    public void runOpMode() {

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
        boolean groupOn = true;
        boolean intakeOn = true;
        boolean launcher = true;
        boolean slowMode = false;

        telemetry.addData("Status", "Initialized and Ready");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            // --- DRIVE INPUTS ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double frontLeftPower = y + x + rx;
            double rearLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double rearRightPower = y + x - rx;

            double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(rearLeftPower),
                    Math.max(Math.abs(frontRightPower), Math.abs(rearRightPower))));
            if (max > 1.0) {
                frontLeftPower /= max;
                rearLeftPower /= max;
                frontRightPower /= max;
                rearRightPower /= max;
            }
            if (gamepad1.xWasPressed()) {
                slowMode = !slowMode;
            }
            if (slowMode) {
                double slowFactor = 0.5;
                frontLeftPower *= slowFactor;
                rearLeftPower *= slowFactor;
                frontRightPower *= slowFactor;
                rearRightPower *= slowFactor;
            }
            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);

            // --- A BUTTON: Toggle Servo Group ---
            if (gamepad1.aWasPressed()) {
                groupOn = !groupOn;
            }

            if (groupOn) {
                one.setPower(1);
                two.setPower(1);
                three.setPower(1);
                four.setPower(-1);
                five.setPower(-1);
                six.setPower(-1);
                zero.setPower(1);
            } else {
                one.setPower(-0.5);
                two.setPower(0);
                three.setPower(0);
                four.setPower(-0.35);
                six.setPower(-0.2);
                zero.setPower(0.4);
                five.setPower(0.5);
            }

            // --- RIGHT BUMPER: Toggle Intake ---
            if (gamepad1.rightBumperWasPressed()) {
                intakeOn = !intakeOn;
            }

            intake.setPower(intakeOn ? -1 : 0);

            // --- LEFT BUMPER: Toggle Launcher ---
            if (gamepad1.leftBumperWasPressed()) {
                launcher = !launcher;
            }

            if (launcher) {
                launchLeft.setVelocity(2150);
                launchRight.setVelocity(2150);
            } else {
                launchLeft.setVelocity(0);
                launchRight.setVelocity(0);
            }

            // --- TELEMETRY ---
            telemetry.clearAll();

            // --- DRIVER ESSENTIALS ---
            telemetry.addLine("=== DRIVER ESSENTIALS ===");
            telemetry.addData("Drive FL/FR/RL/RR", "%.2f %.2f %.2f %.2f",
                    frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
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

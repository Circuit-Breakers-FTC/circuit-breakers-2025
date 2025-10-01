package org.firstinspires.ftc.teamcode.old2024.circuitbreakers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@TeleOp
@Disabled
public class GolfThingy extends LinearOpMode {

    //Set the motors

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo intake;


    @Override
    public void runOpMode() throws InterruptedException {

        //Set Drive Motor
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(Servo.class, "intake");

        long buttonClickTime = 0;
        waitForStart(); //Wait for Opmode Activation
        while (opModeIsActive()) {

            //Driving Options

            double x = gamepad1.left_stick_x; // Strafe left/right
            double y = -gamepad1.left_stick_y; // Forward/backward
            double rotation = gamepad1.right_stick_x; // Rotation

            // Calculate motor powers

            double frontLeftPower = y + x + rotation;
            double frontRightPower = y - x - rotation;
            double backLeftPower = y - x + rotation;
            double backRightPower = y + x - rotation;
            double slowness = (.222);

            //Set Motor Power Limits to 1

            double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                intake.setPosition(0);
                buttonClickTime = System.currentTimeMillis();
            }
            if (System.currentTimeMillis()-buttonClickTime >= 200){
                intake.setPosition(0.7);
            }

            // Set motor powers

            frontLeft.setPower(frontLeftPower * slowness);
            frontRight.setPower(frontRightPower * slowness);
            backLeft.setPower(backLeftPower * slowness);
            backRight.setPower(backRightPower * slowness);
            telemetry.addData("bigPush", intake.getPosition());
            telemetry.update();


        }
    }
}

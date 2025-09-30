package org.firstinspires.ftc.teamcode.old2024.platinum;

import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.ARM_EXTEND_HIGH_BASKET;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.ARM_SCORE_SAMPLE_IN_HIGH;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.ARM_SPEED_UP;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.EXTEND_SPEED;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.INTAKE_DEPOSIT;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.WRIST_FOLDED_IN;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.WRIST_FOLDED_OUT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
@Config
@Autonomous(name="Autonomous Sample", group="Robot")
public class AutonomousSample extends LinearOpMode {
    // Declare motors
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private DcMotor extend = null;
    public CRServo intake      = null; //the active intake servo
    public Servo wrist       = null;
    // Constants for arm positions

    private ElapsedTime runtime = new ElapsedTime();
    // Constants for motor power and timing
    static public double FORWARD_SPEED = 0.25;
    static public double FORWARD_TIME = 1;
    static public int EXTEND_SPECIMEN_POSITION = -1400;
    static public int ARM_SPECIMEN_POSITION = 1200;
    static public double STRAFE_SPEED = 0.25;
    static public double STRAFE_TIME = 2;
    double extendPosition = 0;
    @Override
    public void runOpMode() {
        // Initialize the hardware variables
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        armMotor = hardwareMap.get(DcMotor.class, "arm"); // Update with actual name
        extend = hardwareMap.get(DcMotor.class, "extend"); // Update with actual name
        // Set zero power behavior for arm and extension motors
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        wrist.setPosition(WRIST_FOLDED_IN);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) extend).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        extend.setTargetPosition(0);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motor directions (Reverse motors on one side)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        strafe(STRAFE_SPEED, STRAFE_TIME);
        armMotor.setTargetPosition((int)ARM_SCORE_SAMPLE_IN_HIGH);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(ARM_SPEED_UP);
        wrist.setPosition(WRIST_FOLDED_OUT);
        waitForMotorsToFinish();
        extend.setTargetPosition((int)ARM_EXTEND_HIGH_BASKET);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) extend).setVelocity(EXTEND_SPEED);
        waitForMotorsToFinish();
        drive(FORWARD_SPEED, FORWARD_TIME);
        intake.setPower(INTAKE_DEPOSIT);
        sleep(2000);
        drive(-1*FORWARD_SPEED, FORWARD_TIME);
        extend.setTargetPosition(0);
        waitForMotorsToFinish();
        armMotor.setTargetPosition(0);
        waitForMotorsToFinish();
        telemetry.addData("Done With:", "basket");




//        //extend and lift the arm
//
//        armMotor.setTargetPosition(ARM_SPECIMEN_POSITION);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        ((DcMotorEx) armMotor).setVelocity(ARM_SPEED_UP);
//        waitForMotorsToFinish();
//        wrist.setPosition(WRIST_FOLDED_OUT);
//        sleep(500);
//        waitForMotorsToFinish();
//        extend.setTargetPosition(EXTEND_SPECIMEN_POSITION);
//        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        ((DcMotorEx) extend).setVelocity(EXTEND_SPEED);
//        drive(FORWARD_SPEED, FORWARD_TIME);  // Move the robot for 2.1 seconds
//
//        extend.setTargetPosition(0);
//        waitForMotorsToFinish();
//        armMotor.setTargetPosition(0);
//        waitForMotorsToFinish();
//
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    private void waitForMotorsToFinish() {
        while (extend.isBusy() || armMotor.isBusy()) {
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Extend Position", extend.getCurrentPosition());
            telemetry.update();
        }
    }

    // Helper methods for robot movement
    private void drive(double power, double time) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Moving forward: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        // Arm motor power is not included here to avoid unintended movement
    }

    private void strafe(double power, double time) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Moving forward: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        // Arm motor power is not included here to avoid unintended movement
    }

    private void turnRight(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
    }


}
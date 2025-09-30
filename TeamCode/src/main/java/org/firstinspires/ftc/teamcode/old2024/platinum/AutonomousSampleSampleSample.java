package org.firstinspires.ftc.teamcode.old2024.platinum;

import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.INTAKE_COLLECT;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.INTAKE_OFF;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.WRIST_FOLDED_IN;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.WRIST_FOLDED_OUT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@Autonomous(name="Autonomous Sample & Sample + Sample", group="Robot")
@Disabled
public class AutonomousSampleSampleSample extends PlatinumBase {
    static public double STRAFE_TIME_FIRST = 875;
    static public double STRAFE_TIME_SECOND = 800;
    static public double STRAFE_POWER = 0.4;
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
        intake.setPower(INTAKE_COLLECT);
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
        initAprilTag();
        setManualExposure(WEBCAM_EXPOSURE, WEBCAM_GAIN);  // Use low exposure time to reduce motion blur

        waitForStart();
        wrist.setPosition(WRIST_FOLDED_OUT);
        //extend and lift the arm


        //driveToTag(DESIRED_DISTANCE1, 2.0);
        deposit();
        waitForMotorsToFinish(2);
        driveToTag(DESIRED_DISTANCE1, 2.0);
        driveFromAprilTagToSamplePickupDepositAndGoBack(STRAFE_TIME_FIRST,STRAFE_POWER);
        driveToTag(DESIRED_DISTANCE3, 2.0);
        driveFromAprilTagToSamplePickupDepositAndGoBack(STRAFE_TIME_SECOND,STRAFE_POWER);
        intake.setPower(INTAKE_OFF);
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }



}
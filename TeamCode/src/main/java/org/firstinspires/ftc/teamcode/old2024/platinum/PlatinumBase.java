package org.firstinspires.ftc.teamcode.old2024.platinum;

import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.ARM_EXTEND_HIGH_BASKET;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.ARM_EXTEND_PICK_UP;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.ARM_SCORE_SAMPLE_IN_HIGH;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.ARM_SPEED_UP;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.EXTEND_SPEED;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.INTAKE_COLLECT;
import static org.firstinspires.ftc.teamcode.old2024.platinum.DrivingCrossaints.INTAKE_DEPOSIT;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.ANGLE_GO_SLOW;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.ANGLE_MAX_ERROR;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.MAX_AUTO_SPEED;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.MAX_AUTO_STRAFE;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.MAX_AUTO_TURN;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.MINIMUM_DRIVE_SPEED;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.RANGE_GO_SLOW;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.RANGE_MAX_ERROR;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.SPEED_GAIN;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.STRAFE_GAIN;
import static org.firstinspires.ftc.teamcode.old2024.platinum.RobotAutoDriveToAprilTagOmni.TURN_GAIN;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
public abstract class PlatinumBase extends LinearOpMode {
    // Declare motors
    protected DcMotor leftFrontDrive = null;
    protected DcMotor rightFrontDrive = null;
    protected DcMotor leftBackDrive = null;
    protected DcMotor rightBackDrive = null;
    protected DcMotor armMotor = null;
    protected DcMotor extend = null;
    public CRServo intake      = null; //the active intake servo
    public Servo wrist       = null;
    // Constants for arm positions
    protected VisionPortal visionPortal;               // Used to manage the video source.
    protected AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    protected AprilTagDetection desiredTag = null;
    protected ElapsedTime runtime = new ElapsedTime();
    // Constants for motor power and timing
    static public double FORWARD_SPEED = 0.25;
    static public double FORWARD_TIME = 1.6;
    static public int EXTEND_SPECIMEN_POSITION = -1400;
    static public int ARM_SPECIMEN_POSITION = 1200;

    public static double DESIRED_DISTANCE1 = 24.0; //  this is how close the camera should get to the target (inches)
    public static double DESIRED_DISTANCE2 = 19.0; //
    public static double DESIRED_DISTANCE3 = 15.0; // this is how close the camera should get to the target (inches)
    public static int WEBCAM_EXPOSURE = 3;
    public static int WEBCAM_GAIN = 150;
    public static double TAG_GIVE_UP_TIME = 0.5;
    public static double LINE_UP_TO_BUCKET_POWER = 0.4;
    public static double LINE_UP_TO_BUCKET_TIME = 0.52;
    public static double DRIVE_FORWARD_TO_BUCKET_TIME = 0.45;
    public static double DRIVE_FORWARD_TO_BUCKET_POWER = 0.5;
    public static double INTAKE_DEPOSIT_TIME = 1.0;
    public static double FIRST_ARM_LIFT_WAIT_TIME = 0.1;
    public static double WRIST_FOLD_OUT_WAIT_TIME = 0.5;
    public static double PICKUP_STRAFE_POWER = 0.2;
    public static double PICKUP_STRAFE_TIME = 0.2;
    
   
        // Initialize the hardware variables
       
    public void driveFromAprilTagToSamplePickupDepositAndGoBack(double strafeTime,double strafePower) {
        strafe(strafePower, strafeTime);
        extend.setTargetPosition((int)ARM_EXTEND_PICK_UP); // pickup sample
        betterSleep(0.5);
        armMotor.setTargetPosition((int)ARM_SCORE_SAMPLE_IN_HIGH);
        waitForMotorsToFinish(0.5);
        extend.setTargetPosition((int)ARM_EXTEND_HIGH_BASKET);
        strafe(-1*strafePower, strafeTime);
        driveToTag(DESIRED_DISTANCE2, 2.0);
        turnLeft(LINE_UP_TO_BUCKET_POWER, LINE_UP_TO_BUCKET_TIME); // Line up to bucket
        drive(DRIVE_FORWARD_TO_BUCKET_POWER, DRIVE_FORWARD_TO_BUCKET_TIME);
        betterSleep(1);
        intake.setPower(INTAKE_DEPOSIT); // Deposit sample
        betterSleep(INTAKE_DEPOSIT_TIME);
        drive(-1*DRIVE_FORWARD_TO_BUCKET_POWER, DRIVE_FORWARD_TO_BUCKET_TIME);
        extend.setTargetPosition(0);
        waitForMotorsToFinish(1);
        armMotor.setTargetPosition(0);
        waitForMotorsToFinish(0.5);
        intake.setPower(INTAKE_COLLECT);
        turnLeft(-1*LINE_UP_TO_BUCKET_POWER,LINE_UP_TO_BUCKET_TIME);

    }
    public void deposit() {
        armMotor.setTargetPosition((int)ARM_SCORE_SAMPLE_IN_HIGH);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) armMotor).setVelocity(ARM_SPEED_UP);
        waitForMotorsToFinish(0.5);
        extend.setTargetPosition((int)ARM_EXTEND_HIGH_BASKET);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) extend).setVelocity(EXTEND_SPEED);
        driveToTag(DESIRED_DISTANCE2, 2.0);
        turnLeft(LINE_UP_TO_BUCKET_POWER, LINE_UP_TO_BUCKET_TIME); // Line up to bucket
        drive(DRIVE_FORWARD_TO_BUCKET_POWER, DRIVE_FORWARD_TO_BUCKET_TIME);
        betterSleep(1);
        intake.setPower(INTAKE_DEPOSIT); // Deposit sample
        betterSleep(INTAKE_DEPOSIT_TIME);
        drive(-1*DRIVE_FORWARD_TO_BUCKET_POWER, DRIVE_FORWARD_TO_BUCKET_TIME);
        extend.setTargetPosition(0);
        waitForMotorsToFinish(1);
        armMotor.setTargetPosition(0);
        waitForMotorsToFinish(0.5);
        intake.setPower(INTAKE_COLLECT);
        turnLeft(-1*LINE_UP_TO_BUCKET_POWER,LINE_UP_TO_BUCKET_TIME);


    }
    public void betterSleep(double time) {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time) {
        }
    }

    public AprilTagDetection getDetection() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for(AprilTagDetection detection : currentDetections) {
            if (detection.id == 13 || detection.id == 16) {
                return detection;
            }
        }
        return null;
    }

    public void driveToTag(double desiredDistance, double maxMethodTime) {
        ElapsedTime methodRunTime = new ElapsedTime();
        runtime.reset();
        while (opModeIsActive()) {

            // Tell the driver what we see, and what to do.
            double drive = 0;        // Desired forward power/speed (-1 to +1)
            double strafe = 0;        // Desired strafe power/speed (-1 to +1)
            double turn = 0;        // Desired turning power/speed (-1 to +1)
            desiredTag = getDetection();

            if (desiredTag == null) {
                stopMoving();
                telemetry.addData("Can not see tag for %5.1f seconds", runtime.time());
                telemetry.update();
                if (runtime.time() > TAG_GIVE_UP_TIME) {
                    return;
                } else {
                    continue;
                }
            }
            runtime.reset();
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - desiredDistance);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;
            telemetry.addData("Range Error", "%5.1f inches", rangeError);
            telemetry.addData("Heading Error", "%5.1f degrees", headingError);
            telemetry.addData("Yaw Error", "%5.1f degrees", yawError);

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            if (Math.abs(rangeError) > RANGE_MAX_ERROR && Math.abs(rangeError) < RANGE_GO_SLOW) {
                drive = (Math.abs(rangeError) / rangeError) * MINIMUM_DRIVE_SPEED;
            }
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            if (Math.abs(headingError) > ANGLE_MAX_ERROR && Math.abs(headingError) < ANGLE_GO_SLOW) {
                turn = (Math.abs(headingError) / headingError) * MINIMUM_DRIVE_SPEED;
            }
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            if (Math.abs(yawError) > ANGLE_MAX_ERROR && Math.abs(yawError) < ANGLE_GO_SLOW) {
                strafe = -1 * (Math.abs(yawError) / yawError) * MINIMUM_DRIVE_SPEED;
            }
            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            if(Math.abs(rangeError) <= RANGE_MAX_ERROR && Math.abs(headingError) <= ANGLE_MAX_ERROR && Math.abs(yawError) <= ANGLE_MAX_ERROR){
                stopMoving();
                telemetry.addLine("Reached desired position");
                telemetry.update();
                return;
            }
            if (methodRunTime.time() > maxMethodTime) {
                stopMoving();
                telemetry.addLine("Method timed out.n");
                telemetry.update();
                return;

            }
            moveRobot(drive, strafe, turn);
            telemetry.update();
        }

    }

    protected void waitForMotorsToFinish(double maxTime) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < maxTime && (extend.isBusy() || armMotor.isBusy())) {
            telemetry.addData("Arm Position", armMotor.getCurrentPosition());
            telemetry.addData("Extend Position", extend.getCurrentPosition());
            telemetry.update();
        }
    }

    // Helper methods for robot movement
    protected void drive(double power, double time) {
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Moving forward: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMoving();
    }

    protected void stopMoving() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    protected void strafe(double power, double encoder_ticks) {
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        runtime.reset();
        while (opModeIsActive() &&Math.abs(rightFrontDrive.getCurrentPosition()) < encoder_ticks) {
            telemetry.addData("Path", "strafing forward: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("left front", leftFrontDrive.getCurrentPosition());
            telemetry.addData("right front", rightFrontDrive.getCurrentPosition());
            telemetry.addData("left back", leftBackDrive.getCurrentPosition());
            telemetry.addData("right back", rightBackDrive.getCurrentPosition());
            telemetry.update();
        }
        stopMoving();
    }

    protected void waitUntilAprilTagDetected() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected

        while (opModeIsActive() && targetFound == false) {
            AprilTagDetection detection = getDetection();
            if (detection != null) {
                desiredTag = detection;
                targetFound = true;
            }
        }

    }

    protected void turnRightToAprilTag(double power) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        waitUntilAprilTagDetected();
        stopMoving();
    }
    protected void turnLeft(double power, double time) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "turning: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        stopMoving();
    }
    protected void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (true) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    /*
 Manually set the camera gain and exposure.
 This can only be called AFTER calling initAprilTag(), and only works for Webcams;
*/
    protected void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        DcMotor leftFrontDrive2  = rightBackDrive;
        DcMotor rightFrontDrive2 = leftBackDrive;
        DcMotor leftBackDrive2  = rightFrontDrive;
        DcMotor rightBackDrive2 = leftFrontDrive;


        // Send powers to the wheels.
        leftFrontDrive2.setPower(leftFrontPower);
        rightFrontDrive2.setPower(rightFrontPower);
        leftBackDrive2.setPower(leftBackPower);
        rightBackDrive2.setPower(rightBackPower);
    }

}
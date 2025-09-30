package org.firstinspires.ftc.teamcode.old2024.platinum;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
@TeleOp
@Disabled
public class DrivingCrossaints extends OpMode {

    /* Declare OpMode members. */
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo
    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    public static double ARM_TICKS_PER_DEGREE = 19.7924893140647; //exact fraction is (194481/9826)
    public DcMotor  extend    = null;

    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    public static double ARM_SPEED_UP  = 3000;
    public static double ARM_SPEED_DOWN  = 1200;
    public static double EXTEND_SPEED  = 3000;

    public static double ARM_COLLAPSED_INTO_ROBOT  = 0;
    public static double ARM_COLLECT               = 0;
    public static double ARM_CLEAR_BARRIER         = 700;
    public static double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    public static double ARM_SCORE_SAMPLE_IN_HIGH  = 1800;
    public static double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    public static double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;
    public static double ARM_EXTEND_HIGH_BASKET    = -2100;
    public static double ARM_EXTEND_PICK_UP    = -350;
    public static double ARM_EXTEND_RESET = 0;
    public static double ARM_EXTEND_SUBMERSE = -1200;
    public static double ARM_SUBMERSIBLE = 210;
    public static double ARM_LOW_HANG1 = 1300;//put arm into posion to hang and exstend
    public static double ARM_EXTEND_HANG = -1000 ;             //retract and pull arm down
    public static double ARM_LOW_HANG2 = 1200;           //folds rest of body for full hang
    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    public static double INTAKE_COLLECT    = -1.0;
    public static double INTAKE_OFF        =  0.0;
    public static double INTAKE_DEPOSIT    =  0.25;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    public static double WRIST_FOLDED_IN   = 0;
    public static double WRIST_FOLDED_OUT  = 0.5;
    public static double WRIST_WIGGLE = 0.05;
    public static double WRIST_WIGGLE_TIME_SEC = 1.0;

    /* A number in degrees that the triggers can adjust the arm position by */
    public static double FUDGE_FACTOR = 8 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;
    public double wristPosition = WRIST_FOLDED_IN;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    double extendPosition = 0;
    double armSpeed = ARM_SPEED_UP;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */

        extend = hardwareMap.get(DcMotor.class, "extend"); //the arm motor
        armMotor = hardwareMap.get(DcMotor.class, "arm"); //the arm motor


        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/



        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPosition= armMotor.getCurrentPosition();

        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendPosition = extend.getCurrentPosition();
        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) extend).setCurrentAlert(5, CurrentUnit.AMPS);


        /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
        Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
        If you do not have the encoder plugged into this motor, it will not run in this code. */
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

    }
    boolean isWiggling = false;
    boolean firstLoop = true;
    @Override
    public void loop()  {
        double slowFactor = 0.5;

        if(gamepad2.a){
            armPosition=ARM_LOW_HANG1;
            extendPosition=ARM_EXTEND_HANG;
            intake.setPower(INTAKE_OFF);
            isWiggling=false;
        }
        if(gamepad2.b){
            armPosition=ARM_LOW_HANG2;
            extendPosition=0;
        }
        if(gamepad2.x){
            armPosition=0;
            extendPosition=0;
        }

        if (firstLoop) {
            firstLoop = false;
            wristPosition = WRIST_FOLDED_OUT;
        }
        if (gamepad1.a) {
            intake.setPower(INTAKE_COLLECT);
            isWiggling = true;
        }
        else if (gamepad1.x) {
            intake.setPower(INTAKE_OFF);
            isWiggling = false;
        }
        else if (gamepad1.b) {
            intake.setPower(INTAKE_DEPOSIT);
            isWiggling = false;
        }
        else if (gamepad1.right_bumper) {
            isWiggling = true;
            armPosition = ARM_COLLECT;
            armSpeed = ARM_SPEED_DOWN;
            wristPosition = WRIST_FOLDED_OUT;
            intake.setPower(INTAKE_COLLECT);
            extendPosition = ARM_EXTEND_PICK_UP;
        }

        else if (gamepad1.left_bumper){
            armSpeed = ARM_SPEED_DOWN;
            armPosition = ARM_CLEAR_BARRIER;
            extendPosition = ARM_EXTEND_RESET;
            wristPosition = WRIST_FOLDED_OUT;
        }

        else if (gamepad1.y){
            armPosition = ARM_SCORE_SAMPLE_IN_HIGH;
            armSpeed = ARM_SPEED_UP;
            extendPosition = ARM_EXTEND_HIGH_BASKET;
            wristPosition = WRIST_FOLDED_OUT;
            isWiggling = false;
        }
        else if (gamepad1.dpad_left){
            slowFactor = 1;
        }


        else if (gamepad1.dpad_right){
            /**/
            armPosition = ARM_SUBMERSIBLE;
            armMotor.setTargetPosition((int)ARM_SUBMERSIBLE);
            extendPosition = ARM_EXTEND_SUBMERSE;
            isWiggling = true;
            intake.setPower(INTAKE_COLLECT);
        }

        else if (gamepad1.dpad_up){

            wristPosition = WRIST_FOLDED_OUT;
            isWiggling = false;
        }

        else if (gamepad1.dpad_down){
            /* this moves the arm down to lift the robot up once it has been hooked */
            armPosition = ARM_WINCH_ROBOT;
            intake.setPower(INTAKE_OFF);
            wristPosition = WRIST_FOLDED_IN;
        }

        if (isWiggling){
            double wiggle = WRIST_WIGGLE*Math.cos(System.currentTimeMillis()/1000.0/WRIST_WIGGLE_TIME_SEC*2*Math.PI);
            wrist.setPosition(wristPosition + wiggle);
        }else {
            wrist.setPosition(wristPosition);
        }





            /* Here we set the target position of our arm to match the variable that was selected
            by the driver.
            We also set the target velocity (speed) the motor runs at, and use setMode to run it.*/
        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));
        armMotor.setTargetPosition((int) (armPosition  +armPositionFudgeFactor));
        ((DcMotorEx) armMotor).setVelocity(armSpeed);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        extend.setTargetPosition((int) (extendPosition));
        ((DcMotorEx) extend).setVelocity(EXTEND_SPEED);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /* TECH TIP: Encoders, integers, and doubles
            Encoders report when the motor has moved a specified angle. They send out pulses which
            only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
            position our arm is currently at can be expressed as a whole number of encoder "ticks".
            The encoder will never report a partial number of ticks. So we can store the position in
            an integer (or int).
            A lot of the variables we use in FTC are doubles. These can capture fractions of whole
            numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
            servo power to 0.5.

            setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
            ticks are always whole numbers, it expects an int. But we want to think about our
            arm position in degrees. And we'd like to be able to set it to fractions of a degree.
            So we make our arm positions Doubles. This allows us to precisely multiply together
            armPosition and our armPositionFudgeFactor. But once we're done multiplying these
            variables. We can decide which exact encoder tick we want our motor to go to. We do
            this by "typecasting" our double, into an int. This takes our fractional double and
            rounds it to the nearest whole number.
            */

        /* Check to see if our arm is over the current limit, and report via telemetry. */
        if (((DcMotorEx) armMotor).isOverCurrent()){
            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
        }


        /* send telemetry to the driver of the arm's current position and target position */
        telemetry.addData("armTarget: ", armMotor.getTargetPosition());
        telemetry.addData("arm Encoder: ", armMotor.getCurrentPosition());


        telemetry.addData("extendTarget: ", extend.getTargetPosition());
        telemetry.addData("extend Encoder: ", extend.getCurrentPosition());

        double drive =  gamepad1.left_stick_y; // Forward/Backward
        double strafe = -gamepad1.left_stick_x; // Left/Right
        double rotate = gamepad1.right_stick_x;
        telemetry.addData("drive: ", drive);
        telemetry.addData("strafe: ", strafe);
        telemetry.addData("rotate: ", rotate);
        telemetry.update();

        double frontLeftPower =  drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower =   drive - strafe + rotate;
        double backRightPower =  drive + strafe - rotate;
        if(!gamepad2.b) {
            frontLeftPower = frontLeftPower*slowFactor;
            frontRightPower = frontRightPower*slowFactor;
            backLeftPower = backLeftPower*slowFactor;
            backRightPower = backRightPower*slowFactor;
        }
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }
}
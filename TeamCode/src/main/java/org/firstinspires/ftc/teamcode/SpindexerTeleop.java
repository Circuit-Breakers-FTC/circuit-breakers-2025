package org.firstinspires.ftc.teamcode;

/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "SpindexerTeleop", group = "StarterBot")
//@Disabled
public class SpindexerTeleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.15; //The feeder servos run this long when a shot is requested.
    final double TIME_BETWEEN_SHOTS = 1; //Time between shots.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double gateTime = 1.0;
    double driveTime = 2;
    double speed = 1;
    boolean intakeOn = false;
    boolean yWasPressed = false;
    boolean gatePressed = false;
    boolean sucking = false;
    boolean collectDriving=false;
    int v1=0;
    int pos = 1;
    int targetPos = 0;
    int shotNumber = 0;
    boolean targetPos1 = true;
    boolean targetPos2 = true;
    boolean targetPos3 = true;
    boolean colorLocked = false;
    boolean launchNow= false;
    String shotColor1 = "empty";
    String shotColor2 = "empty";
    String shotColor3 = "empty";
    String comp1 = "green";
    String comp2 = "purple";
    String comp3 = "purple";
    String clrInComp = "empty";
    String pattern = "gpp";
    boolean index = false;


    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    double LAUNCHER_TARGET_VELOCITY;
    double LAUNCHER_MIN_VELOCITY;
    double GATE_OPEN;
    double GATE_CLOSED;

    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotorEx launcher = null;
    private Servo gate = null;
    private DcMotorEx spindexer = null;
    private DcMotor intake = null;
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;

    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime gateTimer = new ElapsedTime();
    ElapsedTime driveTimer = new ElapsedTime();
    ElapsedTime noColorTimer = new ElapsedTime();
    ElapsedTime delayTimer = new ElapsedTime();


    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        ROTATE,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        gate = hardwareMap.get(Servo.class, "gate");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");
        intake = hardwareMap.get(DcMotor.class, "intake");
        colorSensor1 = hardwareMap.get(ColorSensor.class, "color1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color2");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the leftfeeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */


        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        if(gamepad1.dpad_left){
            pattern = "gpp";
        }
        if(gamepad1.dpad_up){
            pattern = "pgp";
        }
        if(gamepad1.dpad_right){
            pattern = "ppg";
        }
        if (pattern == "gpp"){
            shotColor1="green";
            shotColor2="purple";
            shotColor3="purple";
        }
        if (pattern == "pgp"){
            shotColor1="purple";
            shotColor2="green";
            shotColor3="purple";
        }
        if (pattern == "ppg"){
            shotColor1="purple";
            shotColor2="purple";
            shotColor3="green";
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double clrDiv = 1.82;
        boolean colorSeen = false;
        gate.setPosition(0.72);
        if (colorSensor1.alpha()<=colorSensor2.alpha()){
            if (colorSensor1.alpha()<=105) colorSeen = true;
            if (colorSensor1.alpha()>105) colorSeen = false;
        } else {
            if (colorSensor2.alpha()<=105) colorSeen = true;
            if (colorSensor2.alpha()>105) colorSeen = false;
        }
        if(colorSeen){
            if(colorSensor1.alpha()<=colorSensor2.alpha()){
                if(colorSensor2.green()/clrDiv>colorSensor2.red()){
                    clrInComp="green";
                } else if(colorSensor2.green()/clrDiv<colorSensor2.red()){
                    clrInComp="purple";
                }
            } else {
                if(colorSensor1.green()/clrDiv>colorSensor1.red()){
                    clrInComp="green";
                } else if(colorSensor1.green()/clrDiv<colorSensor1.red()){
                    clrInComp="purple";
                }
            }
        } else{
            //clrInComp="empty";
        }
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        LAUNCHER_TARGET_VELOCITY = 1410;
        LAUNCHER_MIN_VELOCITY = 1400;
        boolean index = false;
        intakeOn = true;
        if (gamepad2.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);
        }
        //speed changes
        if (gamepad1.left_trigger>0.5){
            speed=1;
        }

        //medium
        if (gamepad1.leftBumperWasPressed()){
            speed=1.25;
        }
        //slow
        if (gamepad1.rightBumperWasPressed()){
            speed=2.25;
        }

        if (gatePressed) {
            gate.setPosition(0.5);
            if (gateTimer.seconds()>gateTime){
                gate.setPosition(0.75);
                gatePressed=false;
                yWasPressed=true;
            }

        } else if (gamepad1.b) {
            gate.setPosition(0.9);
        }

        if (gamepad1.aWasPressed()) {
            collectDriving=true;
            driveTimer.reset();
        }
        if (collectDriving) {
            if (driveTimer.seconds()<driveTime) {
                leftFrontDrive.setPower(0.25);
                rightFrontDrive.setPower(0.25);
                leftBackDrive.setPower(0.25);
                rightBackDrive.setPower(0.25);
            } else {
                spindexer.setPower(0.0);
                leftFrontDrive.setPower(0.0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0.0);
                rightBackDrive.setPower(0.0);
                collectDriving=false;
            }
        }
        if (colorSeen&&!colorLocked) {
            colorLocked=true;
            noColorTimer.reset();
            delayTimer.reset();
        }
        if (colorLocked && delayTimer.seconds() >= 0.25) {
            if (pos == 1) comp1=clrInComp;
            if (pos == 2) comp2=clrInComp;
            if (pos == 3) comp3=clrInComp;
            pos++;
            if (pos > 3) pos = 1;

            colorLocked = false;
        }
        if (gamepad2.yWasPressed()) {
            yWasPressed = true;
        }
        if (yWasPressed) {
            pos +=1;
            if (pos > 3){
                pos = 1;
            }
            yWasPressed = false;
        }

        if (pos == 1){
            targetPos = 0;
            spindexer.setTargetPosition(targetPos);
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexer.setPower(1.0);
        }
        if (pos == 2){
            targetPos = 180;
            spindexer.setTargetPosition(targetPos);
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexer.setPower(1.0);
        }
        if (pos == 3){
            targetPos = 360;
            spindexer.setTargetPosition(targetPos);
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindexer.setPower(1.0);
        }

        if (intakeOn){
            intake.setPower(-0.9);
        } else {
            intake.setPower(0);
        }
        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad2.rightBumperWasPressed());
        if (gamepad2.rightBumperWasPressed()) {
            launchNow = true;
        }
        if (launchNow) {
            //launch(v1!=3);
        }
        if (index) {
            if (v1==1) {
                if (shotColor1==comp1) {
                    pos=1;
                    comp1="empty";
                } else if (shotColor1==comp2) {
                    pos=2;
                    comp2="empty";
                } else if (shotColor1==comp3) {
                    pos=3;
                    comp3="empty";
                } else {
                    pos=1;
                    comp1="empty";
                }
            }
        }
        int red1 = colorSensor1.red();
        int green1 = colorSensor1.green();
        int blue1 = colorSensor1.blue();
        int alpha1 = colorSensor1.alpha();  // overall brightness
        int red2 = colorSensor2.red();
        int green2 = colorSensor2.green();
        int blue2 = colorSensor2.blue();
        int alpha2 = colorSensor2.alpha();  // overall brightness
        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("gate position", gate.getPosition());
        telemetry.addData("drive timer", driveTimer.seconds());
        telemetry.addData("spindexer pos", spindexer.getTargetPosition());
        telemetry.addData("clrInComp", clrInComp);
        telemetry.addData("comp1", comp1);
        telemetry.addData("comp2", comp2);
        telemetry.addData("comp3", comp3);
        telemetry.addData("shotColor1", shotColor1);
        telemetry.addData("shotColor2", shotColor2);
        telemetry.addData("shotColor3", shotColor3);
        telemetry.addData("pattern", pattern);


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){

        /* the denominator is the largest motor power (absolute value) or 1
         * This ensures all the powers maintain the same ratio,
         * but only if at least one is out of the range [-1, 1]
         */

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), speed);

        leftFrontPower = ((forward + strafe + rotate) / denominator)/speed;
        rightFrontPower = ((forward - strafe - rotate) / denominator)/speed;
        leftBackPower = ((forward - strafe + rotate) / denominator)/speed;
        rightBackPower = ((forward + strafe - rotate) / denominator)/speed;
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(-LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() < -LAUNCHER_MIN_VELOCITY && feederTimer.seconds() > TIME_BETWEEN_SHOTS) {
                    launchState = LaunchState.ROTATE;
                }
                break;
            case ROTATE:
                //index=true;
                launchState = LaunchState.LAUNCH;
            case LAUNCH:
                gateTimer.reset();
                if(v1>0){
                    gatePressed=true;
                }
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    if(v1==3){
                        v1=0;
                    }
                    v1+=1;
                    launchState = LaunchState.IDLE;


                }
                break;
        }
    }
}

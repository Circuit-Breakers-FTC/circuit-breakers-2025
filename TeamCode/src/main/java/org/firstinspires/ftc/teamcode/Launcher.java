package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {

    // Hardware references
    private final DcMotorEx launcherMotor;
    private final Servo gateServo;

    // Constants
    public final double FEED_TIME_SECONDS;
    public final double TIME_BETWEEN_SHOTS;
    public final double LAUNCHER_TARGET_VELOCITY;
    public final double LAUNCHER_MIN_VELOCITY;

    // State variables
    public enum LaunchState { IDLE, SPIN_UP, ROTATE, LAUNCH, LAUNCHING }
    public LaunchState launchState = LaunchState.IDLE;
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();

    private int v1 = 0;
    private boolean gatePressed = false;

    public Launcher(DcMotorEx launcherMotor, Servo gateServo,
                    double targetVelocity, double minVelocity,
                    double feedTime, double timeBetweenShots) {
        this.launcherMotor = launcherMotor;
        this.gateServo = gateServo;
        this.LAUNCHER_TARGET_VELOCITY = targetVelocity;
        this.LAUNCHER_MIN_VELOCITY = minVelocity;
        this.FEED_TIME_SECONDS = feedTime;
        this.TIME_BETWEEN_SHOTS = timeBetweenShots;
    }

    // The shared launch function
    public void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) launchState = LaunchState.SPIN_UP;
                break;

            case SPIN_UP:
                launcherMotor.setVelocity(-LAUNCHER_TARGET_VELOCITY);
                if (launcherMotor.getVelocity() < -LAUNCHER_MIN_VELOCITY &&
                        feederTimer.seconds() > TIME_BETWEEN_SHOTS) {
                    launchState = LaunchState.ROTATE;
                }
                break;

            case ROTATE:
                launchState = LaunchState.LAUNCH;
                break;

            case LAUNCH:
                gateTimer.reset();
                if (v1 > 0) gatePressed = true;
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    if (v1 == 3) v1 = 0;
                    v1 += 1;
                    launchState = LaunchState.IDLE;
                }
                break;

        }
    }

    public boolean isIdle() {
        return launchState == LaunchState.IDLE;
    }


    public void resetV1() { v1 = 0; }

}

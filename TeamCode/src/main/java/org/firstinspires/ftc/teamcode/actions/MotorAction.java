package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorAction implements Action {
    private DcMotor motor;
    private double power;
    private int targetPosition;
    private boolean initialized = false;
    public MotorAction(DcMotor motor, int targetPosition, double power) {
        this.motor = motor;
        this.targetPosition = targetPosition;
        this.power = power;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            initialized = true;
            motor.setTargetPosition(targetPosition);
            motor.setPower(power);
        }
        return motor.isBusy();
    }
}

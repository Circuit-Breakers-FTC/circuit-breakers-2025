package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorActionTargetVelocity implements Action {
    private DcMotorEx motor;
    private double power;
    private double targetVelocity;
    private double accuracy;
    private boolean initialized = false;
    public MotorActionTargetVelocity(DcMotorEx motor, double targetVelocity, double accuracy) {
        this.motor = motor;
        this.targetVelocity = targetVelocity;
        this.accuracy = accuracy;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            initialized = true;
            motor.setVelocity(targetVelocity);
        }
        return (motor.getVelocity() > targetVelocity - accuracy) &&
                (motor.getVelocity() < targetVelocity + accuracy);
    }
}

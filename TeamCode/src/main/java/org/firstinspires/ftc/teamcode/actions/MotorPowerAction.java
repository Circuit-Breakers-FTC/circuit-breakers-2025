package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MotorPowerAction implements Action {
    private DcMotor motor;
    private double power;
    private boolean initialized = false;
    public MotorPowerAction(DcMotor motor, double power) {
        this.motor = motor;
        this.power = power;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            initialized = true;
            motor.setPower(power);
        }
        return false;
    }
}

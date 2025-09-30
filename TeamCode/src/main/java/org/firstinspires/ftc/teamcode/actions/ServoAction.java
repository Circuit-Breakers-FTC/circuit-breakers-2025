package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoAction implements Action {
    private Servo servo;
    private double position;
    private boolean initialized = false;

    public ServoAction(Servo servo, double position) {
        this.servo = servo;
        this.position = position;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            initialized = true;
            servo.setPosition(position);
        }
        return false;
    }
}

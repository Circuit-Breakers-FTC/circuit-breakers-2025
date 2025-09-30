package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.VelConstraint;

public class MaxVelocity implements VelConstraint {
    private double velocity;
    public MaxVelocity(double velocity) {
        this.velocity = velocity;
    }
    @Override
    public double maxRobotVel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
        return this.velocity;
    }
}

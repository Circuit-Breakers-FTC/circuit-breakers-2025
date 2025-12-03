package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;

public class MaxAccel implements AccelConstraint {
    private double value;
    public MaxAccel(double value) {
        this.value = value;
    }
    @NonNull
    @Override
    public MinMax minMaxProfileAccel(@NonNull Pose2dDual<Arclength> pose2dDual, @NonNull PosePath posePath, double v) {
        return new MinMax(-1*this.value, this.value);
    }
}

package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class BlueFlyAuto extends RedFlyAuto {
    @Override public double blueAuto(){
        return -1;
    }
    public static double BLUE_SHOT_ANGLE = -135;
    @Override public double shotAngle() {
        return BLUE_SHOT_ANGLE;
    }
}

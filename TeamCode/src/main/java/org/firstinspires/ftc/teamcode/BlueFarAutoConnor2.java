package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class BlueFarAutoConnor2 extends RedFarAutoConnor2 {
    @Override public double blueAuto(){
        return -1;
    }

    @Override public double blueAutoY(){
        return -6;
    }
}
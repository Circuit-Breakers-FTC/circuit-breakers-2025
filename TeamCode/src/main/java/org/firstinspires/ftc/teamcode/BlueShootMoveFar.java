package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class BlueShootMoveFar extends RedShootMoveFar {
    @Override public double blueAuto(){
        return -1;
    }
}
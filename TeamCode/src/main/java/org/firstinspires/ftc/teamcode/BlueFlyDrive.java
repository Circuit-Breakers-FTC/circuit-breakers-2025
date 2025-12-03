package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlueFlyDrive", group="Linear OpMode")
public class BlueFlyDrive extends RedFlyDrive {
    @Override public double blueAuto(){
        return -1;
    }
}

package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

public class LaunchAction implements Action {

    private final Launcher launcher;
    private boolean initialized = false;

    public LaunchAction(Launcher launcher) {
        this.launcher = launcher;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        launcher.launch(true);
        return launcher.isIdle(); // done when state machine returns to IDLE
    }
}

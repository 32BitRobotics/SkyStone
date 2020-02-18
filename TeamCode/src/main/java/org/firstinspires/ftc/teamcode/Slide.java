package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Slide extends OpBase {
    @Override
    public boolean isAutonomous() {
        return true;
    }

    protected abstract Direction slideDirection();
    protected abstract boolean doWait();

    @Override
    public boolean runRobot() {
        if (doWait()) {
            ElapsedTime et = new ElapsedTime();
            while (opModeIsActive() && et.milliseconds() < 15000);
        }

        runForMS(1500);
        strafe(3100, slideDirection());
        return false;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Autonomous for 2 Seconds")
public class ForwardsFor2Seconds extends OpBase {
    @Override
    public boolean isAutonomous() {
        return true;
    }

    @Override
    public boolean runRobot() {
        all.setPower(0.5);
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.milliseconds() < 2000);
        all.setPower(0);
        return false;
    }
}

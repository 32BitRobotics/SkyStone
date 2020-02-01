package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BackTwoMotors extends OpBase {
    @Override
    public boolean runRobot() {
        leftBack.setPower(0.4);
        rightBack.setPower(0.4);
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.milliseconds() < 10000);
        all.setPower(0);
        return false;
    }

    @Override
    public boolean isAutonomous() {
        return true;
    }
}

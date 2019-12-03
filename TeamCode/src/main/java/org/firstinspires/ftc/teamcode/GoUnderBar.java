package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class GoUnderBar extends OpBase {

    public abstract int sign();

    @Override
    public boolean runRobot() {
        driveInches(12, 12, 0.2, 0.2);

        all.noEncoders();

        ElapsedTime et = new ElapsedTime();

        /*all.setPower(0.2);
        while (opModeIsActive() && et.milliseconds() < 1000);
        all.setPower(0);
        et.reset();
        while (opModeIsActive() && et.milliseconds() < 200);*/

        leftFront.setPower(-0.5 * sign());
        leftBack.setPower(0.5 * sign());
        rightFront.setPower(0.5 * sign());
        rightBack.setPower(-0.5 * sign());

        et.reset();
        while (opModeIsActive() && et.milliseconds() < 3000);
        all.setPower(0);

        return false;
    }

    @Override
    public boolean isAutonomous() {
        return true;
    }
}

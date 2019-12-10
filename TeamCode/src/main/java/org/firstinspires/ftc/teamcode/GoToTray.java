package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class GoToTray extends GoUnderBar {
    @Override
    public void moreStuff() {
        leftFront.setPower(-0.5 * sign());
        leftBack.setPower(0.5 * sign());
        rightFront.setPower(0.5 * sign());
        rightBack.setPower(-0.5 * sign());

        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.milliseconds() < 3000);
        all.setPower(0);
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class NewAgeAutonomous extends OpBase {
    protected abstract boolean doWait();
    protected abstract Side startingSide();
    protected abstract Direction initMoveDirection();

    @Override
    public boolean isAutonomous() {
        return true;
    }

    @Override
    public boolean runRobot() {
        all.noEncoders();

        ElapsedTime et = new ElapsedTime();

        if (doWait()) {
          while (opModeIsActive() && et.milliseconds() < 15000);
        }

        // go out far enough that we don't collide with our partners
        runForMS(1700);

        // strafe far enough that we can reach the tray
        int strafeDuration = startingSide() == Side.Quarry ? 10000 : 950;
        strafe((int)(strafeDuration), initMoveDirection());

        // all the way to the tray
        runForMS(500);

        // set the arm down
        arm.setPower(0.5);
        et.reset();
        while (opModeIsActive() && et.milliseconds() < 500);

        // go backwards for as much time as we have gone forwards
        runForMS(1600, MotorOrientation.Backwards, AutPower * 2);

        et.reset();
        while (opModeIsActive() && et.milliseconds() < 2000);

        // pull the arm up
        arm.setPower(-0.5);
        et.reset();
        while (opModeIsActive() && et.milliseconds() < 500);
        arm.setPower(0);


        // strafe to the opposite of the initial direction until we're not overlapping the tray anymore
        strafe((int)(2500), reverse(initMoveDirection()));
        gyroTurn(reverse(initMoveDirection()), 0, AutPower);

        /*
        // go forwards again so we don't collide with our partner
        runForMS(1700);

        // go under the bar
        // This is a total guess, I have no faith in this number. Please fine tune it.
        strafe(3000, reverse(initMoveDirection()));

:w

         */
        return false;
    }
}

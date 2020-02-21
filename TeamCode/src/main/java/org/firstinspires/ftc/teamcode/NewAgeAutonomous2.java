package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class NewAgeAutonomous2 extends OpBase {
    protected abstract Direction initDirection();

    @Override
    public boolean runRobot() {
        all.resetEncoders();
        all.withEncoders();

        encoderDrive(AutPower, 20, 20, 2);
        int sign = initDirection() == Direction.Left ? 1 : -1;
        encoderDrive(AutPower, -sign * WHEEL_CIRCUMFERENCE / 4.2, sign * WHEEL_CIRCUMFERENCE / 4.2, 30);
        encoderDrive(AutPower, 12, 12, 0.8);
        encoderDrive(AutPower, sign * WHEEL_CIRCUMFERENCE / 4.2, -sign * WHEEL_CIRCUMFERENCE / 4.2, 30);
        encoderDrive(AutPower, 10, 10, 2);

        // set the arm down
        arm.setPower(0.5);
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.milliseconds() < 500);

        encoderDrive(-AutPower * 2, -30, -30, 2);


        return false;
    }

    @Override
    public boolean isAutonomous() {
        return true;
    }
}

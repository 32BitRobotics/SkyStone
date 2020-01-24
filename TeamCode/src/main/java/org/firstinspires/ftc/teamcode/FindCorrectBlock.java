package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class FindCorrectBlock extends OpBase {
    @Override
    public boolean isAutonomous() {
        return true;
    }

    protected abstract Direction initDirection();

    protected boolean getViewAt(Direction dir) {
        return cameraManager.thresh_crosses[
                  initDirection() == Direction.Right ? (dir == Direction.Left ? 0 : 1)
                                                     : (dir == Direction.Left ? 0 : 1)
                ];
    }

    @Override
    public boolean runRobot() {
        // go up to the blocks
        runForMS(1700);

        while (!cameraManager.hasExec ^ opModeIsActive()); // xor

        // strafe to the end of the line of blocks
        // TODO: unreliable number, find a better version of this
        strafe(4000, initDirection());

        // check to see if it's on the opposite one
        // TODO: fine tune this
        if (getViewAt(Direction.Left)) {

        } else {
            setStrafe(reverse(initDirection()));
            while (getViewAt(Direction.Right) && opModeIsActive());
            all.setPower(0);
        }

        // put the arm down
        arm.setPower(0.5);
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.milliseconds() < 500);
        arm.setPower(0);

        // move back
        runForMS(1000, MotorOrientation.Backwards);

        // rotate 90 deg
        // TODO: refactor this
        double firstPos = getZ();
        left.setPower(-0.5);
        right.setPower(0.5);
        while (opModeIsActive() && Math.abs(getZ() - firstPos) < 90);
        all.setPower(0);

        // go under the bar
        runForMS(5000);

        return false;
    }
}

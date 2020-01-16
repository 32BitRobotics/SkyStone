package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class RetrBlocks extends OpBase {

    protected abstract short sign();
    private ElapsedTime et = new ElapsedTime();

    @Override
    public boolean isAutonomous() {
        return true;
    }

    protected void setMoveRight(double power) {
        leftFront.setPower(-power * sign());
        leftBack.setPower(power * sign());
        rightFront.setPower(power * sign());
        rightBack.setPower(-power * sign());
    }

    protected void setMoveLeft(double power) {
        setMoveRight(-power);
    }

    protected void moveRight(double power, int time) {
        setMoveRight(power);

        et.reset();
        while (et.milliseconds() < time && opModeIsActive());
        all.setPower(0);
    }

    protected void moveLeft(double power, int time) {
        moveRight(-power, time);
    }

    private static boolean done(boolean[] blocks) {
        int trueCount = 0;
        for (boolean block : blocks) {
            if (block) trueCount++;
            if (trueCount >= 2) return true;
        }
        return false;
    }

    @Override
    public boolean runRobot() {
        while (!cameraManager.hasExec ^ opModeIsActive()); // xor

        boolean[] blocks = new boolean[5];
        
        // read from camera, go until we can't go no more
        setMoveLeft(0.5);
        while (opModeIsActive() && !cameraManager.thresh_crosses[0] && !cameraManager.thresh_crosses[1]);
        all.setPower(0);

        // find the block
        setMoveRight(0.5);
        while (opModeIsActive() && cameraManager.thresh_crosses[0] && cameraManager.thresh_crosses[1]);
        all.setPower(0);

        // move forwards
        all.setPower(0.5);
        et.reset();
        while (opModeIsActive() && et.milliseconds() < 500);
        all.setPower(0);

        this.arm.setPower(0.3);
        et.reset();
        while (opModeIsActive() && et.milliseconds() < 500);
        arm.setPower(0);

        all.setPower(-0.5);
        et.reset();
        while (opModeIsActive() && et.milliseconds() < 500);
        all.setPower(0);

        double firstPos = getZ();
        left.setPower(-0.5);
        right.setPower(0.5);
        while (opModeIsActive() && Math.abs(getZ() - firstPos) < 90);
        all.setPower(0);

        all.setPower(0.5);
        et.reset();
        while (opModeIsActive() && et.milliseconds() < 2000);
        all.setPower(0);

        return false;
    }
}

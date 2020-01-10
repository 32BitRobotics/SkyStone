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

        // move right until we see yellow in the right
        ElapsedTime et2 = new ElapsedTime();
        setMoveRight(0.5);
        while (opModeIsActive() && !cameraManager.thresh_crosses[0] && cameraManager.thresh_crosses[1]);
        all.setPower(0);

        // if more than 500m has passed, the first block is probably the black one
        int currentPos = 0;
        if (et2.milliseconds() >= 500) {
            currentPos = 1;
        } else {
            // find the block
            setMoveRight(0.5);
            while (opModeIsActive() && !cameraManager.thresh_crosses[0]);
            while (opModeIsActive() && cameraManager.thresh_crosses[0] && cameraManager.thresh_crosses[1]);
            all.setPower(0);
        }

        // the block is on our right half


        return false;
    }
}

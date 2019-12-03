package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Thomas Drive")
public class OpDrive extends OpBase{
    private double x;
    @Override
    public boolean isAutonomous() {
        return false;
    }

    @Override
    public boolean runRobot() {        all.setPower(0);

        if (gamepad1.b) x = 0.5;
        if (Math.abs(gamepad1.left_stick_y) != 0.05) all.setPower(gamepad1.left_stick_y * x);
        if (gamepad1.left_stick_x < 0.05) {
            left.setPower(gamepad1.left_stick_x * x);
            right.setPower(-1 * gamepad1.left_stick_x * x);
        } else if (gamepad1.left_stick_x > 0.05) {
            left.setPower(gamepad1.left_stick_x * x);
            right.setPower(-1 * gamepad1.left_stick_x * x);
        }
        return true;
    }
}
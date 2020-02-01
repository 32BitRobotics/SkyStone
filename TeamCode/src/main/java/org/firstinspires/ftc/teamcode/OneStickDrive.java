package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="One Stick Drive")
public class OneStickDrive extends OpBase {
    private Toggle solenoidToggle = new Toggle();

    @Override
    public boolean runRobot() {
        double left, right;
        double drive, turn, max;

        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;

        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 0.5)
        {
            left /= max;
            right /= max;
        }

        if (gamepad1.a) {
            left *= 0.3;
            right *= 0.3;
        }

        // Output the safe vales to the motor drives.
        this.left.setPower(left);
        this.right.setPower(right);

        if (gamepad1.left_bumper)
            this.arm.setPower(1);
        else if (gamepad1.right_bumper)
            this.arm.setPower(-1);
        else
            this.arm.setPower(0);

        if (gamepad1.left_trigger > 0.5)
            this.lift.setPower(0.5);
        else if (gamepad1.right_trigger > 0.5)
            this.lift.setPower(-0.5);
        else
            this.lift.setPower(0);

        /*solenoidToggle.setInput(gamepad1.x);
        if (solenoidToggle.getValue()) {
            this.solenoids.setPower(1);
        } else {
            this.solenoids.setPower(0);
        }*/

        /*if (gamepad1.left_trigger > 0) {
            claw.setPower(0.2);
        } else if (gamepad1.right_trigger > 0) {
            claw.setPower(-0.2);
        } else claw.setPower(0);*/

        telemetry.clear();
        telemetry.addData("left power", leftFront.getPower());
        telemetry.addData("right power", rightFront.getPower());
        telemetry.update();

        return true;
    }

    @Override
    public boolean isAutonomous() {
        return false;
    }
}

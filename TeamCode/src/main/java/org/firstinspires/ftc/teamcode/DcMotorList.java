package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.security.Policy;
import java.util.ArrayList;

public class DcMotorList extends ArrayList<DcMotor> {
    public void setPower(double power) {
        for (DcMotor motor: this) {
            motor.setPower(power);
        }
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotor motor: this) {
            motor.setMode(runMode);
        }
    }

    public void noEncoders() {
        this.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void withEncoders() {
        this.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoders() {
        this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition() {
        this.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean isBusy() {
        for (DcMotor motor: this) {
            if (motor.isBusy()) return true;
        }
        return false;
    }

    public int getCurrentPosition() {
        int position = this.get(0).getCurrentPosition();
        return position;
    }

    public void setTargetPosition(int val) {
        for (DcMotor motor: this) {
            motor.setTargetPosition(val);
        }
    }

    public void forwards() {
        for (DcMotor motor: this) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public void reverse() {
        for (DcMotor motor: this) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void brakeOnZero() {
        for (DcMotor motor: this) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void coastOnZero() {
        for (DcMotor motor : this) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
}

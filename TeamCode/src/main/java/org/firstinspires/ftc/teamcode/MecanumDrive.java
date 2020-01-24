/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//Declare OpMode Name and Type


public abstract class MecanumDrive extends OpBase{

    private ElapsedTime runtime = new ElapsedTime(); //internal clock
    private Toggle solenoidToggle = new Toggle();

    protected abstract Gamepad armGamepad();

    @Override
    public boolean runRobot() {
        mecanumDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x); //heading drive
        return true;
    }

    @Override
    public boolean isAutonomous() {
        return false;
    }

    /**===============================CLASS METHODS========================================= */

    public void mecanumDrive(double x, double y, double turn) { /** Mecanum Drive System */
        //variables for driving -- Refer to hardware map

        double frontLeftPower = -x+y+turn;
        double frontRightPower = x+y-turn;
        double backLeftPower = x+y+turn;
        double backRightPower = -x+y-turn;

        Double[] speeds = {frontRightPower, frontLeftPower, backLeftPower, backRightPower}; //store speeds in array

        double limit = gamepad1.y ? 1.0 : 0.5;
        for (byte i = 0; i < 4; i++) { //check for clipping in each

            speeds[i]= Range.clip(speeds[i], -limit, limit);
            if (gamepad1.a) speeds[i] *= 0.3;
        }

        //power setting
        this.rightFront.setPower(speeds[0]);
        this.leftFront.setPower(speeds[1]);
        this.leftBack.setPower(speeds[2]);
        this.rightBack.setPower(speeds[3]);

        if (armGamepad().left_bumper)
            this.arm.setPower(1);
        else if (armGamepad().right_bumper)
            this.arm.setPower(-1);
        else
            this.arm.setPower(0);

        /*solenoidToggle.setInput(gamepad1.x);
        if (solenoidToggle.getValue()) {
            this.solenoids.setPower(1);
        } else {
            this.solenoids.setPower(0);
        }*/

        /*if (armGamepad().left_trigger > 0.5) {
            claw.setPower(0.2);
        } else if (armGamepad().right_trigger > 0.5) {
            claw.setPower(-0.2);
        } else claw.setPower(0);*/

        //telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Wheels ", " (%.2f), (%.2f),\n                 (%.2f), (%.2f)", speeds[1], speeds[0], speeds[2], speeds[3]);
        telemetry.update();
    }

}
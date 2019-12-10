package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Mecanum Drive (Same Controller)")
public class MecanumDriveSameController extends MecanumDrive {
    @Override
    protected Gamepad armGamepad() {
        return gamepad1;
    }
}

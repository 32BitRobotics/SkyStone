package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Mecanum Drive (Different Controller)")
public class MecanumDriveDifferentController extends MecanumDrive {
    @Override
    protected Gamepad armGamepad() {
        return gamepad2;
    }
}

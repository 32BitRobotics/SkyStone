package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Encoder Test")
public class EncoderTest extends OpBase {
    @Override
    public boolean runRobot() {
        encoderDrive(0.5, 12, 12, 30);

        return false;
    }

    @Override
    public boolean isAutonomous() {
        return true;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Strafe Test")
public class StrafeTest extends OpBase {
    @Override
    public boolean isAutonomous() {
        return true;
    }

    @Override
    public boolean runRobot() {
        all.noEncoders();
        //strafe(3000, Direction.Left, 1.0);
        //strafe(3000, Direction.Right, 1.0);
        strafe(3000, Direction.Left);
        //gyroTurn(Direction.Left, 0, AutPower);
        strafe(3000, Direction.Right);
        //gyroTurn(Direction.Right, 0, AutPower);
        return false;
    }
}

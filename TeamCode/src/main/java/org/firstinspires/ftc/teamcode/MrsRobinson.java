package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Color Test")
public final class MrsRobinson extends OpBase {
    @Override
    public boolean runRobot() {
        while (opModeIsActive()) {
            telemetry.clear();
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }
        return false;
    }

    @Override
    public boolean isAutonomous() {
        return true;
    }
}

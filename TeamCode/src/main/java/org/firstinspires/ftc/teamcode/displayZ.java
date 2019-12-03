package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Display Z")
public class displayZ extends OpBase{


    @Override
    public boolean runRobot() {
        telemetry.addData("angle",this.getZ());
        return true;
    }

    @Override
    public boolean isAutonomous() {

        return false;
    }
}

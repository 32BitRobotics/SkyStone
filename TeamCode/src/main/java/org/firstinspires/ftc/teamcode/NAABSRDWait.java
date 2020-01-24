package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA-BS-RD-Wait")
public class NAABSRDWait extends NAABSRightDirection {
    @Override
    protected boolean doWait() {
        return true;
    }
}

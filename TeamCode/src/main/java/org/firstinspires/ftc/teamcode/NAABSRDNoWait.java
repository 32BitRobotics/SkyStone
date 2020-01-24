package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA-BS-RD-NoWait")
public class NAABSRDNoWait extends NAABSRightDirection {
    @Override
    protected boolean doWait() {
        return false;
    }
}

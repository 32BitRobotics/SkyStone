package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA-BS-LD-NoWait")
public class NAABSLDNoWait extends NAABSLeftDirection {
    @Override
    protected boolean doWait() {
        return false;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA-BS-LD-Wait")
public class NAABSLDWait extends NAABSLeftDirection {
    @Override
    protected boolean doWait() {
        return true;
    }
}

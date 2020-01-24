package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA-QS-LD-Wait")
public class NAAQSLDWait extends NAAQSLeftDirection {
    @Override
    protected boolean doWait() {
        return true;
    }
}

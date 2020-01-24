package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA-QS-LD-NoWait")
public class NAAQSLDNoWait extends NAAQSLeftDirection {
    @Override
    protected boolean doWait() {
        return false;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA-QS-RD-NoWait")
public class NAAQSRDNoWait extends NAAQSRightDirection {
    @Override
    protected boolean doWait() {
        return false;
    }
}

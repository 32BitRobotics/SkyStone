package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA-QS-RD-Wait")
public class NAAQSRDWait extends NAAQSRightDirection {
    @Override
    protected boolean doWait() {
        return true;
    }
}

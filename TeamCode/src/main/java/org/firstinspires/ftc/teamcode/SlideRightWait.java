package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "S-RD-Wait")
public class SlideRightWait extends SlideRight {
    @Override
    protected boolean doWait() {
        return true;
    }
}

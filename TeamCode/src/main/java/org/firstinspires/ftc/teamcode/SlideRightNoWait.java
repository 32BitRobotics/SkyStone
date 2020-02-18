package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "S-RD-NoWait")
public class SlideRightNoWait extends SlideRight {
    @Override
    protected boolean doWait() {
        return false;
    }
}

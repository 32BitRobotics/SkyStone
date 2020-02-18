package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "S-LD-NoWait")
public class SlideLeftNoWait extends SlideLeft {
    @Override
    protected boolean doWait() {
        return false;
    }
}

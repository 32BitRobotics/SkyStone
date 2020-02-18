package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "S-LD-Wait")
public class SlideLeftWait extends SlideLeft {
    @Override
    protected boolean doWait() {
        return true;
    }
}

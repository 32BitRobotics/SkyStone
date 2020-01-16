package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Go Under Bar Right")
public class GoUnderBarLeft extends GoUnderBar{
    @Override
    public int sign() {
        return -1;
    }

    @Override
    public void moreStuff() {

    }
}

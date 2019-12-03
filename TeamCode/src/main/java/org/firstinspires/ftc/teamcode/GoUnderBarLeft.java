package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Go Under Bar Left")
public class GoUnderBarLeft extends GoUnderBar{
    @Override
    public int sign() {
        return -1;
    }
}

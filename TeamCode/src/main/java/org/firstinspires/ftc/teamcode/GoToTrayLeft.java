package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Go To Tray (Left)")
public class GoToTrayLeft extends GoToTray {
    @Override
    public int sign() {
        return -1;
    }
}

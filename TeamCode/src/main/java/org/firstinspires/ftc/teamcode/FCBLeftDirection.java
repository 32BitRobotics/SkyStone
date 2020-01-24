package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FCB-LD")
public class FCBLeftDirection extends FindCorrectBlock {
    @Override
    protected Direction initDirection() {
        return Direction.Left;
    }
}

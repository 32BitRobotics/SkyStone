package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FCB-RD")
public class FCBRightDirection extends FindCorrectBlock {
    @Override
    protected Direction initDirection() {
        return Direction.Right;
    }
}

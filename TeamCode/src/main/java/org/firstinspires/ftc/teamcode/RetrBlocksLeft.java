package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Retrieve Blocks - Start Left")
public class RetrBlocksLeft extends RetrBlocks {
    @Override
    protected short sign() {
        return -1;
    }
}

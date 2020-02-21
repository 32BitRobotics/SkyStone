package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA2-RD")
public class NAA2RD extends NewAgeAutonomous2 {
    @Override
    protected Direction initDirection() {
        return Direction.Right;
    }
}

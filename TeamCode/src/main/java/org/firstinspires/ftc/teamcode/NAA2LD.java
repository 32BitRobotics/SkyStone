package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "NAA2-LD")
public class NAA2LD extends NewAgeAutonomous2 {
    @Override
    protected Direction initDirection() {
        return Direction.Right;
    }
}

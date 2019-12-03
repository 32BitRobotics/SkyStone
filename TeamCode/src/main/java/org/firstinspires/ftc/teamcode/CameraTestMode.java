package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CameraTestMode extends LinearOpMode {
    CameraManager cm;

    @Override
    public void runOpMode() throws InterruptedException {
        cm = new CameraManager(this.hardwareMap.appContext);
        waitForStart();
        while (opModeIsActive());
    }
}

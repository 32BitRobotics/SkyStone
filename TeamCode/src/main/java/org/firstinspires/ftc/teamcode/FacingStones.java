package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Facing Stones")
public class FacingStones extends OpBase {
    @Override
    public boolean runRobot() {
        ImgProcessor imgProcessor = new ImgProcessor(hardwareMap.appContext);

        // sliiide to the left
        all.noEncoders();
        mecanumDrive(0, 0.5, 0);
        ElapsedTime et = new ElapsedTime();
        while (et.milliseconds() > 3000 && opModeIsActive());
        all.setPower(0);

        all.resetEncoders();
        all.withEncoders();

        gyroDrive(0.4, 12, 0);

        // TODO: image processing

        boolean[] imgPositions = imgProcessor.processImage();

        return false;
    }

    @Override
    public boolean isAutonomous() {
        return true;
    }
}

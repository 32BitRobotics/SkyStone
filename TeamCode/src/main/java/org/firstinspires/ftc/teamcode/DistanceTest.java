package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Distance Test")
public class DistanceTest extends LinearOpMode {
    DistanceSensor distance;

    @Override
    public void runOpMode() throws InterruptedException {
        distance = (DistanceSensor)hardwareMap.get("distance");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.clear();
            telemetry.addData("Distance (in)", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance (cm)", distance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}

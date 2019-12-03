package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Test Motor Named \"Test\"")
public class TestEnocodersTestTest extends LinearOpMode {
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
      motor = hardwareMap.dcMotor.get("test");
      motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      waitForStart();

      motor.setPower(0.3);
      while (opModeIsActive()) {
          telemetry.clear();
          telemetry.addData("Current Position", motor.getCurrentPosition());
          telemetry.update();
      }
    }
}

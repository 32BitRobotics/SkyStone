package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class FindCorrectBlock extends OpBase {
    @Override
    public boolean isAutonomous() {
        return true;
    }

    protected abstract Direction initDirection();

    protected boolean getViewAt(Direction dir) {
        return cameraManager.thresh_crosses[
                  initDirection() == Direction.Right ? (dir == Direction.Left ? 0 : 1)
                                                     : (dir == Direction.Left ? 0 : 1)
                ];
    }

    @Override
    public boolean runRobot() {
        all.resetEncoders();
        all.withEncoders();
        // go up to the blocks
        encoderDrive(AutPower, 28, 28, 3.5);
        all.setPower(AutPower / 5);
        while (opModeIsActive() && distance.getDistance(DistanceUnit.INCH) > 3) {
            telemetry.clear();
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
        all.setPower(0);

        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.milliseconds() < 800);

        // collect thresholds
        final int THRESH_ERROR = 10;
        final int DEFAULT_THRESHOLD = 35 + THRESH_ERROR;
        int cr = color.red(),
            cb = color.blue(),
            cg = color.green();

        int r =/* Math.max(cr, */DEFAULT_THRESHOLD/*)*/ - THRESH_ERROR,
            g = /*Math.max(cg, */DEFAULT_THRESHOLD/*)*/ - THRESH_ERROR,
            b = /*Math.max(cb, */DEFAULT_THRESHOLD/*)*/ - THRESH_ERROR;

        all.noEncoders();
        setStrafe(initDirection());
        et.reset();
        while (opModeIsActive() && color.red() > r && color.green() > g && color.blue() > b) {
            telemetry.clear();
            telemetry.addData("Red", color.red());
            telemetry.addData("Green", color.green());
            telemetry.addData("Blue", color.blue());
            telemetry.update();
        }

        double strafeTime = et.milliseconds();

        et.reset();
        if (strafeTime > 500)
            while (opModeIsActive() && et.milliseconds() < 400);

        all.setPower(0);
        et.reset();
        all.resetEncoders();
        all.withEncoders();
        while (opModeIsActive() && et.milliseconds() < 500);

        encoderDrive(AutPower , 3, 3, 4);
        arm.setPower(0.5);
        et.reset();
        while (opModeIsActive() && et.milliseconds() < 500);
        arm.setPower(0);
        encoderDrive(AutPower * 2, -12, -12, 8);

        // turn 90 deg
        int sign = initDirection() == Direction.Left ? -1 : 1;
        encoderDrive(AutPower, -sign * WHEEL_CIRCUMFERENCE / 4.2, sign * WHEEL_CIRCUMFERENCE / 4.2, 30);

        if (strafeTime < 500) {
            encoderDrive(AutPower * 2, 36, 36, 12);
        } else {
            encoderDrive(AutPower * 2, 48, 48, 12);
        }

        arm.setPower(-0.5);
        et.reset();
        while (opModeIsActive() && et.milliseconds() < 400);
        arm.setPower(0);

        encoderDrive(AutPower * 2, -13, -13, 10);

        return false;
    }
}

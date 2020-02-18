package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public abstract class OpBase extends LinearOpMode {
    public DcMotor leftBack;
    public DcMotor leftFront;
    public DcMotor rightBack;
    public DcMotor rightFront;

    public DcMotorList left;
    public DcMotorList right;
    public DcMotorList all;

    public DcMotor arm;
    public DcMotor lift;

    public ColorSensor color;
    public DistanceSensor distance;

    //public DcMotor leftClaw;
    //public DcMotor rightClaw;
    //public DcMotorList claw = new DcMotorList();

    public GyroSensor gyro;
    Orientation lastAngles = new Orientation();

    enum Side {
        Quarry,
        Build
    }

    enum Direction {
        Left,
        Right
    }

    enum MotorOrientation {
        Forwards,
        Backwards
    }

    public static int motorMultiplier(MotorOrientation orientation) {
        return orientation == MotorOrientation.Forwards ? 1 : -1;
    }

    public static Direction reverse(Direction d) {
        return d == Direction.Left ? Direction.Right : Direction.Left;
    }

    public static final double AutPower = 0.23;

    public void runForMS(int ms, MotorOrientation orientation, double power) {
        all.setPower(power * motorMultiplier(orientation));
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.milliseconds() < ms);
        all.setPower(0);

        telemetry.clear();
        telemetry.addData("Red", color.red());
        telemetry.addData("Green", color.green());
        telemetry.addData("Blue", color.blue());
        telemetry.update();
    }

    public void runForMS(int ms,MotorOrientation orientation) {
       runForMS(ms, orientation, AutPower);
    }

    public void runForMS(int ms) {
        runForMS(ms, MotorOrientation.Forwards);
    }



    public void strafe(int ms, Direction direction, double power) {
        setStrafe(direction, power);

        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.milliseconds() < ms);
        all.setPower(0);
    }

    public void strafe(int ms, Direction direction) {
        strafe(ms, direction, AutPower * 2);
    }

    public void setStrafe(Direction direction, double power) {
        int sign = direction == Direction.Left ? -1 : 1;
        leftFront.setPower(sign * power);
        leftBack.setPower(-sign * power * (direction == Direction.Right ? 0.90 : 0.85));
        rightFront.setPower(-sign * power);
        rightBack.setPower(sign * power);
    }

    public void timeTurn(int ms, Direction direction, double power) {
        int sign = direction == Direction.Left ? -1 : 1;
        left.setPower(sign * power);
        right.setPower(-sign * power);
        ElapsedTime et = new ElapsedTime();
        while (opModeIsActive() && et.milliseconds() < ms);
        all.setPower(0);
    }

    public void timeTurn(int ms, Direction direction) {
        timeTurn(ms, direction, AutPower * 0.75);
    }

    static final double turnTimeout = 5000;

    public void gyroTurn(Direction direction, double angle, double power) {
        return;
        /*
        int sign = direction == Direction.Left ? -1 : 1;
        left.setPower(sign * power);
        right.setPower(-sign * power);
        ElapsedTime et = new ElapsedTime();

        while (opModeIsActive() && et.milliseconds() < turnTimeout) {
            if (sign == -1 && gyro.rawZ() > angle) break;
            else if (sign == 1 && gyro.rawZ() < angle) break;
        }
        all.setPower(0);

         */
    }

    public void setStrafe(Direction direction) {
        setStrafe(direction, AutPower * 2);
    }

    public CameraManager cameraManager;
    public TelemetryManager telemetryManager;

    public abstract boolean runRobot();
    public abstract boolean isAutonomous();

    static final double     COUNTS_PER_MOTOR_REV    = 2240  ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 0.36 * .6 * (22.0/26.0) ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.atan(1) * 4);
    static final double WHEEL_CIRCUMFERENCE = 12.75 * Math.atan(1) * 4 * 2;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize drivetrain
        leftBack = hardwareMap.dcMotor.get("backLeft");
        leftFront = hardwareMap.dcMotor.get("frontLeft");
        rightBack = hardwareMap.dcMotor.get("backRight");
        rightFront = hardwareMap.dcMotor.get("frontRight");

        left = new DcMotorList();
        left.add(leftBack);
        left.add(leftFront);
        right = new DcMotorList();
        right.add(rightFront);
        right.add(rightBack);

        left.forwards();
        right.reverse();

        all = new DcMotorList();
        all.addAll(left);
        all.addAll(right);

        //if (isAutonomous()) { all.resetEncoders(); all.withEncoders(); }
        /*else*/ all.noEncoders();

        // initialize arm
        arm = hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift = hardwareMap.dcMotor.get("lift");
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize solenoids
        /*leftSolenoid = hardwareMap.dcMotor.get("leftSolenoid");
        rightSolenoid = hardwareMap.dcMotor.get("rightSolenoid");
        solenoids = new DcMotorList();
        solenoids.add(leftSolenoid);
        solenoids.add(rightSolenoid);
        solenoids.noEncoders();*/

        // setup claw
        /*leftClaw = hardwareMap.dcMotor.get("leftClaw");
        rightClaw = hardwareMap.dcMotor.get("rightClaw");
        leftClaw.setDirection(DcMotorSimple.Direction.REVERSE);
        rightClaw.setDirection(DcMotorSimple.Direction.FORWARD);
        claw.add(leftClaw);
        claw.add(rightClaw);
        claw.noEncoders();*/

        // initialize gyro
        /*BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hardwareMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);*/
        //gyro = hardwareMap.gyroSensor.get("gyro");
        //gyro.calibrate();
        //gyro.resetZAxisIntegrator();

        // initialize camera
        cameraManager = new CameraManager(hardwareMap.appContext);

        // initialize color sensor
        color = (ColorSensor)hardwareMap.get("color");
        distance = (DistanceSensor)hardwareMap.get("distance");

        // initialize telemetry
        //if (!isAutonomous())
        //  telemetryManager = new TelemetryManager(this);

        waitForStart();

        if (this.isAutonomous()) {
            runRobot();
        }
        else while (opModeIsActive() && runRobot());
    }

    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = left.getCurrentPosition() + moveCounts;
            newRightTarget = right.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);

            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            left.setPower(speed);
            right.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (left.isBusy() && right.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                left.setPower(leftSpeed);
                right.setPower(rightSpeed);
            }

            // Stop all motion;
            all.setPower(0);

            // Turn off RUN_TO_POSITION
            all.withEncoders();
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        all.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        left.setPower(leftSpeed);
        right.setPower(rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getZ();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public double getZ()
    {
        /*double globalAngle = 0;

        //total angle of rotation on Z axis

        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;*/
        return 0;
    }

    public void mecanumDrive(double x, double y, double turn) { /** Mecanum Drive System */
        //variables for driving -- Refer to hardware map

        double frontLeftPower = -x+y+turn;
        double frontRightPower = x+y-turn;
        double backLeftPower = x+y+turn;
        double backRightPower = -x+y-turn;

        Double[] speeds = {frontRightPower, frontLeftPower, backLeftPower, backRightPower}; //store speeds in array

        for (byte i = 0; i < 4; i++) { //check for clipping in each
            speeds[i]= Range.clip(speeds[i], -1.0, 1.0);
        }

        //power setting
        this.rightFront.setPower(speeds[0]);
        this.leftFront.setPower(speeds[1]);
        this.leftBack.setPower(speeds[2]);
        this.rightBack.setPower(speeds[3]);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        ElapsedTime runtime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            left.setTargetPosition(newLeftTarget);
            right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left.setPower(Math.abs(speed));
            right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot wil// always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left.isBusy() && right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        left.getCurrentPosition(),
                        right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            left.setPower(0);
            right.setPower(0);

            // Turn off RUN_TO_POSITION
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    protected void driveInches(double leftinch, double rightinch, double leftspeed, double rightspeed) {
        //all.resetEncoders();
        //all.withEncoders();

        double fixed = 3000;
        /*double leftBackTarget = leftBack.getCurrentPosition() + (leftinch * COUNTS_PER_INCH);
        double leftFrontTarget = leftFront.getCurrentPosition() + (leftinch * COUNTS_PER_INCH);
        double rightBackTarget = rightBack.getCurrentPosition() + (rightinch + COUNTS_PER_INCH);
        double rightFrontTarget = rightFront.getCurrentPosition() + (rightinch * COUNTS_PER_INCH);*/
        double leftBackTarget = fixed, leftFrontTarget = fixed, rightBackTarget = fixed, rightFrontTarget = fixed;

        leftBack.setPower(leftspeed);
        leftFront.setPower(leftspeed);
        right.setPower(rightspeed);

        while (((leftBack.getCurrentPosition() < leftBackTarget || rightBack.getCurrentPosition() < rightBackTarget) ||
               (leftFront.getCurrentPosition() < leftFrontTarget || rightFront.getCurrentPosition() < rightFrontTarget))
                       && opModeIsActive()) {
          if (leftBack.getCurrentPosition() >= leftBackTarget) leftBack.setPower(0);
          if (rightBack.getCurrentPosition() >= rightBackTarget) rightBack.setPower(0);
          if (leftFront.getCurrentPosition() >= leftBackTarget) leftFront.setPower(0);
          if (rightFront.getCurrentPosition() >= rightBackTarget) rightFront.setPower(0);

          telemetry.clear();
          telemetry.addData("left back position", leftBack.getCurrentPosition());
          telemetry.addData("right back position", rightBack.getCurrentPosition());
          telemetry.addData("left front position", leftFront.getCurrentPosition());
          telemetry.addData("right front position", rightFront.getCurrentPosition());
          telemetry.addData("left back target", leftBackTarget);
          telemetry.addData("right back target", rightBackTarget);
          telemetry.addData("left front target", leftFrontTarget);
          telemetry.addData("right front target", rightFrontTarget);
          telemetry.update();
        }

        telemetry.clear();
        telemetry.addData("all done", true);
        telemetry.update();

        all.setPower(0);
    }


}

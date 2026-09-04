package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class BasicDrivetrain {
    public enum Motor {
        LEFT_MOTOR,
        RIGHT_MOTOR
    }

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private LinearOpMode opMode;
    private final ElapsedTime runtime = new ElapsedTime();
    private double leftPower = 0.0;
    private double rightPower = 0.0;

    private static final double COUNTS_PER_MOTOR_REV = 560.0;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;
    private static final double TRACK_WIDTH_INCHES = 16.0;
    private static final double SPEED_UP_RATE = 2.75;
    private static final double SLOW_DOWN_RATE = 5.50;
    private static final double TURN_CIRCUMFERENCE = Math.PI * TRACK_WIDTH_INCHES;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

    /**
     * A replacement for the constructor of this class (making a custom method allows more functionalities than constructor).
     * NEEDS TO BE CALLED EVERY TIME THIS CLASS IS INSTANTIATED.
     * @param opMode Pass in "this" in the OpMode. It will give the OpMode object,
     *               allowing this class to use things like telemetry
     * @param hardwareMap Pass in hardwareMap in the OpMode, letting this class access the maps of the motors on the control hub.
     */
    public void init(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // Motors are mounted opposite each other.
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
    }

    public void setDrivePower(double leftPower, double rightPower) {
        setPower(Motor.LEFT_MOTOR, leftPower);
        setPower(Motor.RIGHT_MOTOR, rightPower);
    }

    /**
     * Gradually changes both motor powers toward the requested powers.
     * @param wantedLeftPower The requested left motor power.
     * @param wantedRightPower The requested right motor power.
     * @param loopTime The seconds since the previous control loop.
     */
    public void setSmoothDrivePower(double wantedLeftPower, double wantedRightPower, double loopTime) {
        double newLeftPower = smoothPower(leftPower, wantedLeftPower, loopTime);
        double newRightPower = smoothPower(rightPower, wantedRightPower, loopTime);

        setDrivePower(newLeftPower, newRightPower);
    }

    /**
     * Gradually changes motor power instead of changing it instantly.
     */
    private double smoothPower(double currentPower, double wantedPower, double loopTime) {
        boolean isChangingDirection = currentPower != 0.0
                && wantedPower != 0.0
                && Math.signum(currentPower) != Math.signum(wantedPower);

        boolean isSlowingDown = Math.abs(wantedPower) < Math.abs(currentPower);

        double rate = isChangingDirection || isSlowingDown
                ? SLOW_DOWN_RATE
                : SPEED_UP_RATE;

        return moveToward(currentPower, wantedPower, rate * loopTime);
    }

    /**
     * Moves a value toward a target by no more than maximumChange.
     */
    private double moveToward(double current, double target, double maximumChange) {
        double change = Range.clip(target - current, -maximumChange, maximumChange);
        return current + change;
    }

    /**
     * Drives each side a specified distance using encoders.
     * Positive distances move forward; negative distances move backward.
     * @param speed The speed at which to drive forward.
     * @param leftInches The distance to move the left side of the robot.
     * @param rightInches The distance to move the right side of the robot.
     * @param timeoutSeconds The amount of time after which to abort the movement even if it is not done.
     */
    public void driveInches(double speed, double leftInches, double rightInches, double timeoutSeconds) {
        if (!opMode.opModeIsActive()) {
            return;
        }

        double drivePower = Range.clip(Math.abs(speed), 0.0, 1.0);

        if (drivePower == 0.0 || timeoutSeconds <= 0.0) {
            stop();
            return;
        }

        int leftTarget = getCurrentPosition(Motor.LEFT_MOTOR) + inchesToTicks(leftInches);
        int rightTarget = getCurrentPosition(Motor.RIGHT_MOTOR) + inchesToTicks(rightInches);

        setTargetPosition(Motor.LEFT_MOTOR, leftTarget);
        setTargetPosition(Motor.RIGHT_MOTOR, rightTarget);

        setMode(Motor.LEFT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
        setMode(Motor.RIGHT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        try {
            setDrivePower(drivePower, drivePower);

            // Wait until both motors finish or the timeout expires.
            while (opMode.opModeIsActive()
                    && runtime.seconds() < timeoutSeconds
                    && (isBusy(Motor.LEFT_MOTOR) || isBusy(Motor.RIGHT_MOTOR))) {

                opMode.telemetry.addData("Target", "Left: %d  Right: %d", newLeftTarget, newRightTarget);
                opMode.telemetry.addData("Position", "Left: %d  Right: %d", leftMotor.getCurrentPosition(), rightMotor.getCurrentPosition());
                opMode.telemetry.addData("Time", "%.1f / %.1f seconds", runtime.seconds(), timeoutSeconds);
                opMode.telemetry.update();
                opMode.idle();
            }
        }
        finally {
            stop();
            setMode(Motor.LEFT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            setMode(Motor.RIGHT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Turns the robot. Positive degrees turn clockwise.
     */
    public void turnDegrees(double speed, double degrees, double timeoutSeconds) {
        double inches = (degrees / 360.0) * TURN_CIRCUMFERENCE;
        driveInches(speed, inches, -inches, timeoutSeconds);
    }

    public double getPower(Motor motor) {
        switch (motor) {
            case RIGHT_MOTOR: return rightMotor.getPower();
            case LEFT_MOTOR: return leftMotor.getPower();
            default: return 0;
        }
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public boolean isDriving() {
        return isBusy(Motor.LEFT_MOTOR) || isBusy(Motor.RIGHT_MOTOR);
    }

    public void resetEncoders() {
        stop();

        setMode(Motor.LEFT_MOTOR, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(Motor.RIGHT_MOTOR, DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMode(Motor.LEFT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
        setMode(Motor.RIGHT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Stops all the motors and sets their speeds to 0
     */
    public void stop() {
        leftPower = 0.0;
        rightPower = 0.0;

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    public void setMotorSpeed(Motor motor, double speed) {
        setPower(motor, speed);
    }

    public int getCurrentPosition(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR: return leftMotor.getCurrentPosition();
            case RIGHT_MOTOR: return rightMotor.getCurrentPosition();
            default: return 0;
        }
    }

    public void setTargetPosition(Motor motor, int target) {
        getMotor(motor).setTargetPosition(target);
    }

    public void setPower(Motor motor, double power) {
        double clippedPower = Range.clip(power, -1.0, 1.0);
        switch (motor) {
            case LEFT_MOTOR: leftPower = clippedPower; break;
            case RIGHT_MOTOR: rightPower = clippedPower; break;
        }

        getMotor(motor).setPower(clippedPower);
    }

    public void setMode(Motor motor, DcMotor.RunMode mode) {
        getMotor(motor).setMode(mode);
    }

    public boolean isBusy(Motor motor) {
         return getMotor(motor).isBusy();

    }

    /**
     * Gradually changes motor power instead of changing it instantly.
     */
    private double smoothPower(
            double currentPower,
            double wantedPower,
            double loopTime
    ) {
        boolean changingDirection = currentPower != 0.0
                && wantedPower != 0.0
                && Math.signum(currentPower) != Math.signum(wantedPower);

        boolean slowingDown = Math.abs(wantedPower) < Math.abs(currentPower);

        double rate = changingDirection || slowingDown
                ? SLOW_DOWN_RATE
                : SPEED_UP_RATE;

        return moveToward(currentPower, wantedPower, rate * loopTime);
    }

    /**
     * Moves a value toward a target by no more than maximumChange.
     */
    private double moveToward(double current, double target, double maximumChange) {
        double change = Range.clip(target - current, -maximumChange, maximumChange);
        return current + change;
    }

    private DcMotor getMotor(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR:
                return leftMotor;

            case RIGHT_MOTOR:
                return rightMotor;

            default:
                throw new IllegalArgumentException("Unknown drivetrain motor");
        }
    }

    private int inchesToTicks(double inches) {
        return (int) Math.round(inches * COUNTS_PER_INCH);
    }
}

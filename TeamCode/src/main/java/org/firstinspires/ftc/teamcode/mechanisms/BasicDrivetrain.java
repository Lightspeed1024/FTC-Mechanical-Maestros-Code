package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class BasicDrivetrain {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private LinearOpMode opMode;

    private final ElapsedTime runtime = new ElapsedTime();

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    public enum Motor {
        LEFT_MOTOR,
        RIGHT_MOTOR
    }

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
        this.leftPower = Range.clip(leftPower, -1.0, 1.0);
        this.rightPower = Range.clip(rightPower, -1.0, 1.0);

        leftMotor.setPower(this.leftPower);
        rightMotor.setPower(this.rightPower);
    }

    /**
     * Gradually changes both motor powers toward the requested powers.
     *
     * @param wantedLeftPower requested left motor power
     * @param wantedRightPower requested right motor power
     * @param loopTime seconds since the previous control loop
     */
    public void setSmoothDrivePower(
            double wantedLeftPower,
            double wantedRightPower,
            double loopTime
    ) {
        double newLeftPower = smoothPower(leftPower, wantedLeftPower, loopTime);
        double newRightPower = smoothPower(rightPower, wantedRightPower, loopTime);

        setDrivePower(newLeftPower, newRightPower);
    }

    /**
     * Drives each side a specified distance using encoders.
     * Positive distances move forward; negative distances move backward.
     */
    public void driveInches(
            double speed,
            double leftInches,
            double rightInches,
            double timeoutSeconds
    ) {
        if (!opMode.opModeIsActive()) {
            return;
        }

        double drivePower = Range.clip(Math.abs(speed), 0.0, 1.0);

        if (drivePower == 0.0 || timeoutSeconds <= 0.0) {
            stop();
            return;
        }

        int newLeftTarget = leftMotor.getCurrentPosition() + inchesToTicks(leftInches);
        int newRightTarget = rightMotor.getCurrentPosition() + inchesToTicks(rightInches);

        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        try {
            leftMotor.setPower(drivePower);
            rightMotor.setPower(drivePower);

            // Wait until both motors finish or the timeout expires.
            while (opMode.opModeIsActive()
                    && runtime.seconds() < timeoutSeconds
                    && (leftMotor.isBusy() || rightMotor.isBusy())) {

                opMode.telemetry.addData(
                        "Target",
                        "Left: %d  Right: %d",
                        newLeftTarget,
                        newRightTarget
                );

                opMode.telemetry.addData(
                        "Position",
                        "Left: %d  Right: %d",
                        getLeftTicks(),
                        getRightTicks()
                );

                opMode.telemetry.addData(
                        "Time",
                        "%.1f / %.1f seconds",
                        runtime.seconds(),
                        timeoutSeconds
                );

                opMode.telemetry.update();
                opMode.idle();
            }
        } finally {
            stop();

            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveStraight(double speed, double inches, double timeoutSeconds) {
        driveInches(speed, inches, inches, timeoutSeconds);
    }

    /**
     * Turns using encoder distances. Positive degrees turn clockwise.
     */
    public void turnDegrees(double speed, double degrees, double timeoutSeconds) {
        double inches = (degrees / 360.0) * TURN_CIRCUMFERENCE;
        driveInches(speed, inches, -inches, timeoutSeconds);
    }

    public int getLeftTicks() {
        return leftMotor.getCurrentPosition();
    }

    public int getRightTicks() {
        return rightMotor.getCurrentPosition();
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public boolean isDriving() {
        return leftMotor.isBusy() || rightMotor.isBusy();
    }

    public void resetEncoders() {
        stop();

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void stop() {
        leftPower = 0.0;
        rightPower = 0.0;

        if (leftMotor != null) {
            leftMotor.setPower(0.0);
        }

        if (rightMotor != null) {
            rightMotor.setPower(0.0);
        }
    }

    public void setMotorSpeed(Motor motor, double speed) {
        setPower(motor, speed);
    }

    public int getCurrentPosition(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR:
                return leftMotor.getCurrentPosition();

            case RIGHT_MOTOR:
                return rightMotor.getCurrentPosition();

            default:
                return 0;
        }
    }

    public void setTargetPosition(Motor motor, int target) {
        switch (motor) {
            case LEFT_MOTOR:
                leftMotor.setTargetPosition(target);
                break;

            case RIGHT_MOTOR:
                rightMotor.setTargetPosition(target);
                break;
        }
    }

    public void setPower(Motor motor, double power) {
        power = Range.clip(power, -1.0, 1.0);

        switch (motor) {
            case LEFT_MOTOR:
                leftMotor.setPower(power);
                break;

            case RIGHT_MOTOR:
                rightMotor.setPower(power);
                break;
        }
    }

    public void setMode(Motor motor, DcMotor.RunMode mode) {
        switch (motor) {
            case LEFT_MOTOR:
                leftMotor.setMode(mode);
                break;

            case RIGHT_MOTOR:
                rightMotor.setMode(mode);
                break;
        }
    }

    public boolean isBusy(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR:
                return leftMotor.isBusy();

            case RIGHT_MOTOR:
                return rightMotor.isBusy();

            default:
                return false;
        }
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

    private int inchesToTicks(double inches) {
        return (int) Math.round(inches * COUNTS_PER_INCH);
    }
}

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

    // Change these measurements if the motors, wheels, or gearing change.
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
     * Connects the code to the drivetrain motors and prepares them for driving.
     */
    public void init(LinearOpMode opMode, HardwareMap hardwareMap) {
        this.opMode = opMode;

        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        // The motors face opposite directions, so the right motor is reversed.
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
     * Changes motor power gradually so the robot does not jerk forward or backward.
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
     * Uses the motor encoders to move each side of the robot a set distance.
     * Positive distances move forward, and negative distances move backward.
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

        int leftTarget = getCurrentPosition(Motor.LEFT_MOTOR) + inchesToTicks(leftInches);
        int rightTarget = getCurrentPosition(Motor.RIGHT_MOTOR) + inchesToTicks(rightInches);

        setTargetPosition(Motor.LEFT_MOTOR, leftTarget);
        setTargetPosition(Motor.RIGHT_MOTOR, rightTarget);

        setMode(Motor.LEFT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);
        setMode(Motor.RIGHT_MOTOR, DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();

        try {
            setDrivePower(drivePower, drivePower);

            // Wait until both motors finish, time runs out, or the OpMode stops.
            while (opMode.opModeIsActive()
                    && runtime.seconds() < timeoutSeconds
                    && (isBusy(Motor.LEFT_MOTOR) || isBusy(Motor.RIGHT_MOTOR))) {

                opMode.telemetry.addData(
                        "Target",
                        "Left: %d  Right: %d",
                        leftTarget,
                        rightTarget
                );

                opMode.telemetry.addData(
                        "Position",
                        "Left: %d  Right: %d",
                        getCurrentPosition(Motor.LEFT_MOTOR),
                        getCurrentPosition(Motor.RIGHT_MOTOR)
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

            setMode(Motor.LEFT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            setMode(Motor.RIGHT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Turns the robot using encoder distances. Positive degrees turn clockwise.
     */
    public void turnDegrees(double speed, double degrees, double timeoutSeconds) {
        double inches = (degrees / 360.0) * TURN_CIRCUMFERENCE;
        driveInches(speed, inches, -inches, timeoutSeconds);
    }

    public double getPower(Motor motor) {
        switch (motor) {
            case LEFT_MOTOR:
                return leftPower;

            case RIGHT_MOTOR:
                return rightPower;

            default:
                return 0.0;
        }
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
        double clippedPower = Range.clip(power, -1.0, 1.0);

        switch (motor) {
            case LEFT_MOTOR:
                leftPower = clippedPower;
                leftMotor.setPower(clippedPower);
                break;

            case RIGHT_MOTOR:
                rightPower = clippedPower;
                rightMotor.setPower(clippedPower);
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
     * Changes the motor power gradually instead of changing it all at once.
     */
    private double smoothPower(double currentPower, double wantedPower, double loopTime) {
        boolean isChangingDirection = currentPower != 0.0
                && wantedPower != 0.0
                && Math.signum(currentPower) != Math.signum(wantedPower);

        // Slow the motor to zero before making it spin in the opposite direction.
        if (isChangingDirection) {
            return moveToward(currentPower, 0.0, SLOW_DOWN_RATE * loopTime);
        }

        boolean isSlowingDown = Math.abs(wantedPower) < Math.abs(currentPower);
        double rate = isSlowingDown ? SLOW_DOWN_RATE : SPEED_UP_RATE;

        return moveToward(currentPower, wantedPower, rate * loopTime);
    }

    /**
     * Moves a value toward its target without changing it too quickly.
     */
    private double moveToward(double current, double target, double maximumChange) {
        double change = Range.clip(target - current, -maximumChange, maximumChange);
        return current + change;
    }

    /**
     * Converts a distance in inches into motor encoder ticks.
     */
    private int inchesToTicks(double inches) {
        return (int) Math.round(inches * COUNTS_PER_INCH);
    }
}

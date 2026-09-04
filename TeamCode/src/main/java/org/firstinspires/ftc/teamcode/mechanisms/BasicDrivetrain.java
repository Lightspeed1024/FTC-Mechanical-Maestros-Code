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

    public enum Motor {
        LEFT_MOTOR,
        RIGHT_MOTOR
    }

 private static final double COUNTS_PER_MOTOR_REV = 560.0;
 private static final double DRIVE_GEAR_REDUCTION = 1.0;
 private static final double WHEEL_DIAMETER_INCHES = 3.54331;
 private static final double TRACK_WIDTH_INCHES = 16.0;

 private static final double TURN_CIRCUMFERENCE = Math.PI * TRACK_WIDTH_INCHES;

 private static final double COUNTS_PER_INCH =
        (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                / (WHEEL_DIAMETER_INCHES * Math.PI);

    /**
     * Finds and configures the drivetrain motors.
     */
    public void init(
            LinearOpMode opMode,
            HardwareMap hardwareMap
    ) {
        this.opMode = opMode;

        leftMotor = hardwareMap.get(
                DcMotor.class,
                "leftMotor"
        );

        rightMotor = hardwareMap.get(
                DcMotor.class,
                "rightMotor"
        );

        /*
         * Reverse one motor because the drivetrain motors are normally
         * mounted as mirror images of each other.
         */
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        /*
         * BRAKE helps the robot stop instead of freely coasting.
         */
        leftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        rightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        resetEncoders();
    }

    /**
     * Sets the power of both drivetrain motors.
     *
     * This method is used by BasicTeleOp.
     */
    public void setDrivePower(
            double leftPower,
            double rightPower
    ) {
        leftPower = Range.clip(
                leftPower,
                -1.0,
                1.0
        );

        rightPower = Range.clip(
                rightPower,
                -1.0,
                1.0
        );

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    /**
     * Drives each side a specified number of inches.
     *
     * Positive distance drives forward.
     * Negative distance drives backward.
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

        double drivePower = Range.clip(
                Math.abs(speed),
                0.0,
                1.0
        );

        /*
         * Avoid waiting for the entire timeout if no usable command
         * was provided.
         */
        if (drivePower == 0.0 || timeoutSeconds <= 0.0) {
            stop();
            return;
        }

        int newLeftTarget =
                leftMotor.getCurrentPosition()
                        + inchesToTicks(leftInches);

        int newRightTarget =
                rightMotor.getCurrentPosition()
                        + inchesToTicks(rightInches);

        leftMotor.setTargetPosition(newLeftTarget);
        rightMotor.setTargetPosition(newRightTarget);

        leftMotor.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        rightMotor.setMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        runtime.reset();

        try {
            /*
             * RUN_TO_POSITION determines the required direction from
             * each motor's current position and target position.
             */
            leftMotor.setPower(drivePower);
            rightMotor.setPower(drivePower);

            /*
             * OR keeps the loop running until both motors finish.
             * The timeout prevents the loop from running forever if
             * one motor becomes stuck.
             */
            while (opMode.opModeIsActive()
                    && runtime.seconds() < timeoutSeconds
                    && (leftMotor.isBusy()
                    || rightMotor.isBusy())) {

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

            /*
             * Return the motors to the mode used by TeleOp.
             */
            leftMotor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );

            rightMotor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER
            );
        }
    }

    /**
     * Drives both sides the same distance.
     */
    public void driveStraight(
            double speed,
            double inches,
            double timeoutSeconds
    ) {
        driveInches(
                speed,
                inches,
                inches,
                timeoutSeconds
        );
    }

    /**
     * Turns the robot using encoder distances.
     *
     * Positive degrees turn clockwise.
     * Negative degrees turn counterclockwise.
     */
    public void turnDegrees(
            double speed,
            double degrees,
            double timeoutSeconds
    ) {
        double inches =
                (degrees / 360.0)
                        * turnCircumference;

        driveInches(
                speed,
                inches,
                -inches,
                timeoutSeconds
        );
    }

    /**
     * Returns the left encoder position.
     *
     * This method is used by BasicTeleOp telemetry.
     */
    public int getLeftTicks() {
        return leftMotor.getCurrentPosition();
    }

    /**
     * Returns the right encoder position.
     *
     * This method is used by BasicTeleOp telemetry.
     */
    public int getRightTicks() {
        return rightMotor.getCurrentPosition();
    }

    /**
     * Returns true while either motor is moving toward a target.
     */
    public boolean isDriving() {
        return leftMotor.isBusy()
                || rightMotor.isBusy();
    }

    /**
     * Resets both drivetrain encoders.
     */
    public void resetEncoders() {
        stop();

        leftMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        /*
         * This must happen after STOP_AND_RESET_ENCODER.
         * Otherwise, the motors remain in reset mode.
         */
        leftMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        rightMotor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );
    }

    /**
     * Immediately sets both motor powers to zero.
     */
    public void stop() {
        if (leftMotor != null) {
            leftMotor.setPower(0.0);
        }

        if (rightMotor != null) {
            rightMotor.setPower(0.0);
        }
    }

    /*
     * The following methods allow autonomous programs to control
     * an individual motor when needed.
     */

    public void setMotorSpeed(
            Motor motor,
            double speed
    ) {
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

    public void setTargetPosition(
            Motor motor,
            int target
    ) {
        switch (motor) {
            case LEFT_MOTOR:
                leftMotor.setTargetPosition(target);
                break;

            case RIGHT_MOTOR:
                rightMotor.setTargetPosition(target);
                break;
        }
    }

    public void setPower(
            Motor motor,
            double power
    ) {
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

    public void setMode(
            Motor motor,
            DcMotor.RunMode mode
    ) {
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
     * Converts inches into the nearest whole encoder count.
     */
    private int inchesToTicks(double inches) {
        return (int) Math.round(
                inches * countsPerInch
        );
    }
}

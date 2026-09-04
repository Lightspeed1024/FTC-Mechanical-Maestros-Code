package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BasicDrivetrain;

@TeleOp(name = "Basic TeleOp", group = "Drive")
public class BasicTeleOp extends LinearOpMode {
    private final BasicDrivetrain drivetrain = new BasicDrivetrain();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private static final double NORMAL_SPEED = 0.65;
    private static final double FAST_SPEED = 1.0;
    private static final double SLOW_SPEED = 0.35;
    private static final double DEAD_ZONE = 0.06;
    private static final double TURN_SPEED = 0.80;
    private static final double FAST_TURN_SPEED = 0.55; // the turning speed when driving at full speed
    private static final double MAXIMUM_SPEED_UP_RATE = 2.75;
    private static final double MAXIMUM_SLOW_DOWN_RATE = 5.50;

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap);

        telemetry.addLine("Robot is ready");
        telemetry.addLine("Left stick: drive");
        telemetry.addLine("Right stick: turn");
        telemetry.addLine("Right trigger: boost to 90%");
        telemetry.addLine("Left bumper: 35% slow mode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            drivetrain.stop();
            return;
        }

        loopTimer.reset();

        try {
            while (opModeIsActive()) {
                double loopTime = Math.min(loopTimer.seconds(), 0.10);
                loopTimer.reset();

                double drive = fixJoystick(-gamepad1.left_stick_y); // y value is reversed in FTC gamepad mapping
                double turn = fixJoystick(gamepad1.right_stick_x); // if robot turns the wrong direction, add minus sign in front

                //cubing for more control at lower speeds but still with same max speed
                drive = drive * drive * drive;
                turn = turn * turn * turn;

                boolean slowMode = gamepad1.left_bumper;
                double boostAmount = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

                // The right trigger gradually increases the maximum drivetrain power from 65% to 90%.
                double speedLimit = mix(NORMAL_SPEED, FAST_SPEED, boostAmount);

                // slow mode would override right trigger boost
                if (slowMode) {
                    speedLimit = SLOW_SPEED;
                }

                // reduce turning sensitivity when driving quickly
                double turnLimit = mix(TURN_SPEED, FAST_TURN_SPEED, Math.abs(drive));

                turn *= turnLimit;

                double wantedLeftPower = drive + turn;
                double wantedRightPower = drive - turn;

                /*
                 * Normalize both motor powers if either one is outside
                 * the allowed range. This preserves their relative ratio.
                 */
                double biggestPower = Math.max(Math.abs(wantedLeftPower), Math.abs(wantedRightPower));

                if (biggestPower > 1.0) {
                    wantedLeftPower /= biggestPower;
                    wantedRightPower /= biggestPower;
                }

                wantedLeftPower *= speedLimit;
                wantedRightPower *= speedLimit;

                leftPower = smoothPower(leftPower, wantedLeftPower, loopTime);
                rightPower = smoothPower(rightPower, wantedRightPower, loopTime);
                drivetrain.setDrivePower(leftPower, rightPower);

                String driveMode;

                if (slowMode) {
                    driveMode = "SLOW";
                }
                else if (boostAmount > 0.05) {
                    driveMode = "BOOST";
                }
                else {
                    driveMode = "NORMAL";
                }

                telemetry.addData("Drive Mode", driveMode);
                telemetry.addData("Speed Limit", "%.0f%%", speedLimit * 100.0);
                telemetry.addData("Right Trigger", "%.0f%%", boostAmount * 100.0);
                telemetry.addData("Motor Power", "Left: %.2f  Right: %.2f", leftPower, rightPower);
                telemetry.addData("Encoders", "Left: %d  Right: %d", drivetrain.getLeftTicks(), drivetrain.getRightTicks());
                telemetry.update();
                idle();
            }
        }
        finally {
            /*
             * Always stop the motors when the OpMode ends, including
             * when an interruption or error occurs.
             */
            drivetrain.stop();
        }
    }

    /**
     * Normalizes the y-stick reading to account for the middle dead-zone
     * @param stickValue The current y-axis reading
     * @return The normalized value of the y-axis
     */
    private double fixJoystick(double stickValue) {
        double amount = Math.abs(stickValue);
        if (amount <= DEAD_ZONE) {
            return 0.0;
        }

        /*
         * Remove the dead zone and rescale the remaining range so the
         * joystick can still produce a value of 1.0.
         */
        double fixedAmount = (amount - DEAD_ZONE) / (1.0 - DEAD_ZONE);
        return Math.copySign(fixedAmount, stickValue);
    }

    /**
     * Smoothly applies power to the motors by using the moveToward function internally but with smoothed values.
     * It
     * @param currentPower The power of the motor at the moment
     * @param wantedPower The desired power of the motor
     * @param loopTime The amount of time the last loop took
     * @return The value of the moveToward function
     */
    private double smoothPower(double currentPower, double wantedPower, double loopTime) {
        boolean changingDirection =
                currentPower != 0.0
                        && wantedPower != 0.0
                        && Math.signum(currentPower) != Math.signum(wantedPower);
        /*
         * When reversing direction, first bring the motor to zero.
         * This prevents the power from crossing directly from forward into reverse during one loop.
         */
        if (changingDirection) {
            return moveToward(currentPower, 0.0, MAXIMUM_SLOW_DOWN_RATE * loopTime);
        }

        boolean slowingDown = Math.abs(wantedPower) < Math.abs(currentPower);
        double maxRate = slowingDown ? MAXIMUM_SLOW_DOWN_RATE : MAXIMUM_SPEED_UP_RATE;

        return moveToward(currentPower, wantedPower, maxRate * loopTime);
    }

    /**
     * Adds the targetValue to the currentValue, but clipped at maximumChange
     * @param currentValue
     * @param targetValue
     * @param maximumChange
     * @return The new value
     */
    private double moveToward(double currentValue, double targetValue, double maximumChange) {
        double change = Range.clip(targetValue - currentValue, -maximumChange, maximumChange);
        return currentValue + change;
    }

    /**
     * A linear interpolation method that smoothly moves the start value towards the end value by a certain percent
     * @param start The start value
     * @param end The end value
     * @param amount The percent to move from start to end, WRITTEN AS A DECIMAL
     * @return The new value
     */
    private double mix(double start, double end, double amount) {
        amount = Range.clip(amount, 0.0, 1.0);
        return start + (end - start) * amount;
    }
}

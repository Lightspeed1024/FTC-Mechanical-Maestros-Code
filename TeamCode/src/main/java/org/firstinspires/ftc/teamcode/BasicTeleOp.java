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

    private static final double normalSpeed = 0.65;
    private static final double fastSpeed = 0.90;
    private static final double slowSpeed = 0.35;
    private static final double deadZone = 0.06;

    private static final double turnSpeed = 0.80;
    private static final double fastTurnSpeed = 0.55;

    private static final double speedUpRate = 2.75;
    private static final double slowDownRate = 5.50;

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
                double loopTime = Math.min(
                        loopTimer.seconds(),
                        0.10
                );

                loopTimer.reset();

                /*
                 * FTC reports forward movement of the left stick as a
                 * negative value, so the Y value must be reversed.
                 */
                double drive = fixJoystick(
                        -gamepad1.left_stick_y
                );

                /*
                 * Positive right-stick X should make the robot turn right.
                 * If the robot turns in the opposite direction, reverse
                 * this value by adding a minus sign.
                 */
                double turn = fixJoystick(
                        gamepad1.right_stick_x
                );

                /*
                 * Cubing the values provides gentle movement near the
                 * center while still allowing maximum power.
                 */
                drive = drive * drive * drive;
                turn = turn * turn * turn;

                boolean slowMode = gamepad1.left_bumper;

                double boostAmount = Range.clip(
                        gamepad1.right_trigger,
                        0.0,
                        1.0
                );

                /*
                 * The right trigger gradually increases the maximum
                 * drivetrain power from 65% to 90%.
                 */
                double speedLimit = mix(
                        normalSpeed,
                        fastSpeed,
                        boostAmount
                );

                /*
                 * Slow mode overrides the right-trigger boost.
                 */
                if (slowMode) {
                    speedLimit = slowSpeed;
                }

                /*
                 * Reduce turning sensitivity while driving quickly.
                 */
                double turnLimit = mix(
                        turnSpeed,
                        fastTurnSpeed,
                        Math.abs(drive)
                );

                turn *= turnLimit;

                double wantedLeftPower = drive + turn;
                double wantedRightPower = drive - turn;

                /*
                 * Normalize both motor powers if either one is outside
                 * the allowed range. This preserves their relative ratio.
                 */
                double biggestPower = Math.max(
                        Math.abs(wantedLeftPower),
                        Math.abs(wantedRightPower)
                );

                if (biggestPower > 1.0) {
                    wantedLeftPower /= biggestPower;
                    wantedRightPower /= biggestPower;
                }

                wantedLeftPower *= speedLimit;
                wantedRightPower *= speedLimit;

                leftPower = smoothPower(
                        leftPower,
                        wantedLeftPower,
                        loopTime
                );

                rightPower = smoothPower(
                        rightPower,
                        wantedRightPower,
                        loopTime
                );

                drivetrain.setDrivePower(
                        leftPower,
                        rightPower
                );

                String driveMode;

                if (slowMode) {
                    driveMode = "SLOW";
                } else if (boostAmount > 0.05) {
                    driveMode = "BOOST";
                } else {
                    driveMode = "NORMAL";
                }

                telemetry.addData(
                        "Drive Mode",
                        driveMode
                );

                telemetry.addData(
                        "Speed Limit",
                        "%.0f%%",
                        speedLimit * 100.0
                );

                telemetry.addData(
                        "Right Trigger",
                        "%.0f%%",
                        boostAmount * 100.0
                );

                telemetry.addData(
                        "Motor Power",
                        "Left: %.2f  Right: %.2f",
                        leftPower,
                        rightPower
                );

                telemetry.addData(
                        "Encoders",
                        "Left: %d  Right: %d",
                        drivetrain.getLeftTicks(),
                        drivetrain.getRightTicks()
                );

                telemetry.update();
                idle();
            }
        } finally {
            /*
             * Always stop the motors when the OpMode ends, including
             * when an interruption or error occurs.
             */
            drivetrain.stop();
        }
    }

    private double fixJoystick(double stickValue) {
        double amount = Math.abs(stickValue);

        if (amount <= deadZone) {
            return 0.0;
        }

        /*
         * Remove the dead zone and rescale the remaining range so the
         * joystick can still produce a value of 1.0.
         */
        double fixedAmount =
                (amount - deadZone)
                        / (1.0 - deadZone);

        return Math.copySign(
                fixedAmount,
                stickValue
        );
    }

    private double smoothPower(
            double currentPower,
            double wantedPower,
            double loopTime
    ) {
        boolean changingDirection =
                currentPower != 0.0
                        && wantedPower != 0.0
                        && Math.signum(currentPower)
                        != Math.signum(wantedPower);

        /*
         * When reversing direction, first bring the motor to zero.
         * This prevents the power from crossing directly from forward
         * into reverse during one loop.
         */
        if (changingDirection) {
            return moveToward(
                    currentPower,
                    0.0,
                    slowDownRate * loopTime
            );
        }

        boolean slowingDown =
                Math.abs(wantedPower)
                        < Math.abs(currentPower);

        double rate = slowingDown
                ? slowDownRate
                : speedUpRate;

        return moveToward(
                currentPower,
                wantedPower,
                rate * loopTime
        );
    }

    private double moveToward(
            double currentValue,
            double targetValue,
            double maximumChange
    ) {
        double change = Range.clip(
                targetValue - currentValue,
                -maximumChange,
                maximumChange
        );

        return currentValue + change;
    }

    private double mix(
            double start,
            double end,
            double amount
    ) {
        amount = Range.clip(
                amount,
                0.0,
                1.0
        );

        return start + (end - start) * amount;
    }
}

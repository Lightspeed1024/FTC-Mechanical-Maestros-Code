package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mechanisms.BasicDrivetrain;
import org.firstinspires.ftc.teamcode.mechanisms.BasicDrivetrain.Motor;

@TeleOp(name = "Basic TeleOp", group = "Drive")
public class BasicTeleOp extends LinearOpMode {
    private final BasicDrivetrain drivetrain = new BasicDrivetrain();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private static final double NORMAL_SPEED = 0.75;
    private static final double FAST_SPEED = 1.00;
    private static final double SLOW_SPEED = 0.35;
    private static final double DEAD_ZONE = 0.06;

    private static final double TURN_SPEED = 0.80;
    private static final double MOVING_TURN_SPEED = 0.55;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this, hardwareMap);

        telemetry.addLine("Robot is ready");
        telemetry.addLine("Left stick: drive");
        telemetry.addLine("Right stick: turn");
        telemetry.addLine("Normal speed: 75%");
        telemetry.addLine("Right trigger: boost to 100%");
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

                // The Y value is reversed in FTC gamepad mapping.
                double drive = fixJoystick(-gamepad1.left_stick_y);
                double turn = fixJoystick(gamepad1.right_stick_x);

                // Cubing gives more control at low speeds without reducing maximum speed.
                drive = drive * drive * drive;
                turn = turn * turn * turn;

                boolean slowMode = gamepad1.left_bumper;
                double boostAmount = Range.clip(gamepad1.right_trigger, 0.0, 1.0);

                // The trigger gradually increases the speed limit from 75% to 100%.
                double speedLimit = interpolate(NORMAL_SPEED, FAST_SPEED, boostAmount);

                if (slowMode) {
                    speedLimit = SLOW_SPEED;
                }

                // Reduce turning sensitivity while driving quickly.
                double turnLimit = interpolate(TURN_SPEED, MOVING_TURN_SPEED, Math.abs(drive));
                turn *= turnLimit;

                double wantedLeftPower = drive + turn;
                double wantedRightPower = drive - turn;

                // Keep both powers in range without changing their relative ratio.
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

                drivetrain.setSmoothDrivePower(wantedLeftPower, wantedRightPower, loopTime);

                String driveMode;

                if (slowMode) {
                    driveMode = "SLOW";
                } else if (boostAmount > 0.05) {
                    driveMode = "BOOST";
                } else {
                    driveMode = "NORMAL";
                }

                telemetry.addData("Drive Mode", driveMode);
                telemetry.addData("Speed Limit", "%.0f%%", speedLimit * 100.0);
                telemetry.addData("Right Trigger", "%.0f%%", boostAmount * 100.0);

                telemetry.addData(
                        "Motor Power",
                        "Left: %.2f  Right: %.2f",
                        drivetrain.getPower(Motor.LEFT_MOTOR),
                        drivetrain.getPower(Motor.RIGHT_MOTOR)
                );

                telemetry.addData(
                        "Encoders",
                        "Left: %d  Right: %d",
                        drivetrain.getCurrentPosition(Motor.LEFT_MOTOR),
                        drivetrain.getCurrentPosition(Motor.RIGHT_MOTOR)
                );

                telemetry.update();
                idle();
            }
        } finally {
            drivetrain.stop();
        }
    }

    /**
     * Removes the joystick dead zone while preserving its full output range.
     */
    private double fixJoystick(double stickValue) {
        double amount = Math.abs(stickValue);

        if (amount <= DEAD_ZONE) {
            return 0.0;
        }

        double fixedAmount = (amount - DEAD_ZONE) / (1.0 - DEAD_ZONE);
        return Math.copySign(fixedAmount, stickValue);
    }

    /**
     * Calculates a value between start and end.
     *
     * An amount of 0 returns start, 1 returns end, and 0.5 returns
     * the value halfway between them.
     */
    private double interpolate(double start, double end, double amount) {
        amount = Range.clip(amount, 0.0, 1.0);
        return start + (end - start) * amount;
    }
}
